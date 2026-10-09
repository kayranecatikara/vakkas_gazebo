#!/usr/bin/env python3
"""
Ground-truth target detector for SITL.

Subscribes to Gazebo poses (dynamic_pose/info, 250 Hz) of the interceptor
and the Talon.  Each tick:

1. Compute the Talon position relative to the interceptor's nose camera.
2. If the Talon is inside the camera FOV and closer than --max-range,
   project it to get a bounding box and compute line-of-sight, range,
   and target attitude — all in the camera optical frame (x right, y down,
   z along the optical axis).
3. Pack a binary target_vision packet (see target_vision_protocol.h) and
   send it over UDP to the target_vision driver at --rate Hz.

No noise or detection latency is added; this is a perfect detector for
algorithm development.  Robustness tests add noise later (step 8).

    python3 tools/sim_detector.py                        # defaults
    python3 tools/sim_detector.py --rate 30 --port 15600

Mirrored wire format: src/drivers/target_vision/target_vision_protocol.h
"""

import argparse
import math
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
import re
import socket
import struct
import time

import numpy as np
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.double_pb2 import Double
from gz.msgs10.double_v_pb2 import Double_V
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
MODEL_DIR = os.path.join(SIM_DIR, "models", "talon1718")

# ----------------------------------------------------------------- protocol ---

MAGIC = b"TV"
VERSION = 1
PAYLOAD_SIZE = 64
PACKET_SIZE = 70
CRC_OFFSET = 2
CRC_LENGTH = 66  # from version to end of bbox (66 bytes)

FLAG_DETECTED = 1 << 0
FLAG_RANGE    = 1 << 1
FLAG_ATTITUDE = 1 << 2
FLAG_BODY_LOS = 1 << 3


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    """CRC-16-CCITT (poly 0x1021) matching target_vision_crc16 in the driver."""
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def pack_packet(frame_id, latency_us, flags, confidence,
                los, range_m, range_sigma_m, q, bbox):
    """Build a 70-byte target_vision packet."""
    # header: magic (2), version (1), payload_len (1)
    hdr = MAGIC + struct.pack("<BB", VERSION, PAYLOAD_SIZE)
    # payload: 64 bytes
    payload = struct.pack("<II", frame_id, latency_us)
    payload += struct.pack("<BB", flags, confidence)
    payload += struct.pack("<H", 0)  # reserved
    payload += struct.pack("<3f", *los)
    payload += struct.pack("<f", range_m)
    payload += struct.pack("<f", range_sigma_m)
    payload += struct.pack("<4f", *q)
    payload += struct.pack("<4f", *bbox)
    assert len(payload) == PAYLOAD_SIZE
    body = hdr + payload  # 68 bytes before CRC
    crc = crc16_ccitt(body[CRC_OFFSET:])  # version through bbox
    return body + struct.pack("<H", crc)


# --------------------------------------------------------------- geometry ---

def quat_to_rot(w, x, y, z):
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])


def rot_to_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return w, x, y, z


# gz camera frame (+x forward, +y left, +z up) -> camera optical frame (+x right, +y down, +z forward)
GZ_TO_OPT = np.array([
    [ 0., -1.,  0.],
    [ 0.,  0., -1.],
    [ 1.,  0.,  0.]
])


def read_stl_bounds(path):
    """Read an STL and return all unique vertices."""
    with open(path, "rb") as f:
        f.seek(80)
        n = struct.unpack("<I", f.read(4))[0]
        rec = np.frombuffer(f.read(n * 50),
                            dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return rec["v"].reshape(-1, 3).astype(np.float64)


def talon_vertices():
    """All visual vertices of the Talon model, in its body frame."""
    sdf = open(os.path.join(MODEL_DIR, "model.sdf")).read()
    link_pose = {m.group(1): np.array([float(v) for v in m.group(2).split()[:3]])
                 for m in re.finditer(r'<link name="(\w+)">\s*<pose>([^<]+)</pose>', sdf)}
    pts = []
    for link in re.finditer(r'<link name="(\w+)">(.*?)</link>', sdf, re.S):
        offset = link_pose.get(link.group(1), np.zeros(3))
        for uri in re.findall(r"<uri>model://talon1718/meshes/([^<]+)</uri>", link.group(2)):
            v = np.unique(np.round(read_stl_bounds(os.path.join(MODEL_DIR, "meshes", uri)), 3), axis=0)
            pts.append(v + offset)
    return np.concatenate(pts)


# -------------------------------------------------------------------- main ---

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="ankara")
    ap.add_argument("--interceptor", default="interceptor_0",
                    help="interceptor model name in Gazebo")
    ap.add_argument("--target", default="talon1718_1",
                    help="target model name in Gazebo")
    ap.add_argument("--port", type=int, default=15600,
                    help="UDP port of the target_vision driver")
    ap.add_argument("--rate", type=float, default=30.0,
                    help="detection rate [Hz]")
    ap.add_argument("--max-range", type=float, default=75.0,
                    help="max detection range [m]")
    ap.add_argument("--hfov", type=float, default=None,
                    help="camera HFOV [deg], default: from interceptor.yaml")
    ap.add_argument("--width", type=int, default=None,
                    help="image width [px], default: from interceptor.yaml")
    ap.add_argument("--height", type=int, default=None,
                    help="image height [px], default: from interceptor.yaml")
    a = ap.parse_args()

    # camera intrinsics: from interceptor.yaml unless overridden
    import yaml
    with open(os.path.join(SIM_DIR, "interceptor.yaml")) as f:
        cfg = yaml.safe_load(f)
    cam = cfg.get("camera", {})
    width  = a.width  or cam.get("width", 1280)
    height = a.height or cam.get("height", 720)
    hfov   = a.hfov   or cam.get("hfov_deg", 60.0)
    hfov_rad = math.radians(hfov)
    vfov_rad = 2 * math.atan(math.tan(hfov_rad / 2) * height / width)
    fx = width  / 2 / math.tan(hfov_rad / 2)
    fy = fx  # square pixels
    cx, cy = width / 2, height / 2

    # Talon vertices for bounding box projection
    try:
        talon_pts = talon_vertices()
        print(f"Talon model: {len(talon_pts)} vertices")
    except Exception as e:
        print(f"Warning: could not load Talon model ({e}), using point-source bbox")
        talon_pts = None

    # gz camera sensor pose relative to the interceptor body:
    # In SDF, the sensor pose is <pose>0 0 {nose} 0 -1.5708 0</pose> (pitched -90° in body).
    # In Gazebo, +x is the camera sensor's forward optical axis.
    # Since the camera looks out of the interceptor nose (+z in body FLU),
    # body +z must map to gz-camera +x.
    # Therefore, the rotation taking body-frame vectors into gz-camera vectors is:
    R_cam_gz_body = np.array([
        [ 0., 0., 1.],
        [ 0., 1., 0.],
        [-1., 0., 0.]
    ])

    # camera offset in the body frame (approximately at the nose)
    fus_len = cfg["fuselage"]["length"]
    # CG is computed in build_interceptor.py; approximate it from the components
    components = cfg.get("components", {})
    total_mass = cfg["fuselage"]["mass"]
    cg_num = 0.0
    for name, comp in components.items():
        total_mass += comp["mass"]
        cg_num += comp["mass"] * comp["z"]
    cg_num += cfg["fuselage"]["mass"] * fus_len / 2
    cg = cg_num / total_mass
    cam_offset_body = np.array([0.0, 0.0, fus_len - cg])  # nose in body FLU (z up)

    # gz transport
    node = Node()
    state = {"poses": {}, "t": 0.0}

    def on_clock(msg):
        state["t"] = msg.sim.sec + msg.sim.nsec * 1e-9

    def on_pose(msg):
        for p in msg.pose:
            if p.name in (a.interceptor, a.target):
                state["poses"][p.name] = p
            elif "gimbal_pitch_link" in p.name:
                state["poses"]["gimbal_pitch_link"] = p

    node.subscribe(Clock, f"/world/{a.world}/clock", on_clock)
    node.subscribe(Pose_V, f"/world/{a.world}/dynamic_pose/info", on_pose)

    # UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = ("127.0.0.1", a.port)

    # 2-axis gimbal setup
    gimbal_cfg = cfg.get("gimbal", {})
    gimbal_enabled = gimbal_cfg.get("enabled", False)
    if gimbal_enabled:
        pub_gimbal_yaw = node.advertise(f"/model/{a.interceptor}/command/seeker_yaw", Double)
        pub_gimbal_pitch = node.advertise(f"/model/{a.interceptor}/command/seeker_pitch", Double)
        yaw_limit = math.radians(gimbal_cfg.get("yaw_limit_deg", 45))
        pitch_limit = math.radians(gimbal_cfg.get("pitch_limit_deg", 45))
        print(f"sim_detector: 2-axis seeker gimbal active (+/-{math.degrees(yaw_limit):.0f}° yaw, "
              f"+/-{math.degrees(pitch_limit):.0f}° pitch)")
    else:
        pub_gimbal_yaw = None
        pub_gimbal_pitch = None
        yaw_limit = pitch_limit = 0.0

    pub_detection = node.advertise(f"/model/{a.interceptor}/detection", Double_V)

    print(f"sim_detector: {a.interceptor} camera → {a.target}, "
          f"udp {a.port}, {a.rate:.0f} Hz, max range {a.max_range:.0f} m, "
          f"FOV {hfov:.0f}°×{math.degrees(vfov_rad):.0f}°, "
          f"{width}×{height}")

    # wait for both models
    while a.interceptor not in state["poses"] or a.target not in state["poses"]:
        time.sleep(0.1)
    print("Both models found, starting detection loop", flush=True)

    frame_id = 0
    rate_far = 50.0   # > 20m: 1920x1200 @ 50 FPS
    rate_near = 80.0  # <= 20m: 960x600 @ 80 FPS
    current_rate = rate_far
    current_mode_near = False
    dt = 1.0 / current_rate
    last_send = 0.0
    last_sim_time = 0.0
    last_print = 0.0

    # 2-Axis Kinematic Seeker Gimbal state (physical rate limits)
    gmb_psi = 0.0      # current yaw / azimuth angle [rad]
    gmb_theta = 0.0    # current pitch / elevation angle [rad]
    # Physical maximum slew rate (default 90 deg/s)
    max_rate_rad = math.radians(gimbal_cfg.get("max_rate_deg_s", 90.0))
    # Optical visual servoing tracking state
    last_visible = False
    last_bbox = None
    last_seen_time = 0.0

    while True:
        time.sleep(0.001)
        t_sim = state["t"]

        if t_sim - last_send < dt:
            continue

        dt_step = max(0.001, min(0.1, t_sim - last_sim_time)) if last_sim_time > 0 else dt
        last_sim_time = t_sim
        last_send = t_sim
        frame_id += 1

        # get poses
        pi = state["poses"].get(a.interceptor)
        pt = state["poses"].get(a.target)

        if pi is None or pt is None:
            continue

        # interceptor pose in world
        R_int = quat_to_rot(pi.orientation.w, pi.orientation.x,
                            pi.orientation.y, pi.orientation.z)
        p_int = np.array([pi.position.x, pi.position.y, pi.position.z])

        # target pose in world
        R_tgt = quat_to_rot(pt.orientation.w, pt.orientation.x,
                            pt.orientation.y, pt.orientation.z)
        p_tgt = np.array([pt.position.x, pt.position.y, pt.position.z])

        # Relative target vector from interceptor nose
        p_nose_world = p_int + R_int @ cam_offset_body
        d_tgt_world = p_tgt - p_nose_world
        d_tgt_body = R_int.T @ d_tgt_world  # FLU (+x fwd, +y left, +z up nose)
        dist_tgt = float(np.linalg.norm(d_tgt_body))

        # Kinematic Seeker Gimbal: Pitch-Only Active Tracking (Yaw locked to 0 boresight)
        if gimbal_enabled and pub_gimbal_yaw is not None and pub_gimbal_pitch is not None:
            # 1. Yaw axis locked firmly to 0 boresight (azimuth handled by drone heading)
            gmb_psi = 0.0
            msg_y = Double()
            msg_y.data = 0.0
            pub_gimbal_yaw.publish(msg_y)

            # 2. Pitch axis active tracking: points camera optical axis directly at target in body elevation plane
            if dist_tgt > 0.5:
                theta_desired = float(np.clip(math.atan2(d_tgt_body[0], d_tgt_body[2]), -pitch_limit, pitch_limit))
            else:
                # Target lost: smoothly return to neutral 0 boresight
                theta_desired = 0.0

            # Enforce physical maximum angular velocity (slew-rate limiter <= 90 deg/s)
            max_delta = max_rate_rad * dt_step
            d_theta = float(np.clip(theta_desired - gmb_theta, -max_delta, max_delta))

            # Exponential low-pass smoothing (critically damped servo response, tau ~ 0.06s)
            alpha = 1.0 - math.exp(-dt_step / 0.06)
            gmb_theta += d_theta * (0.4 + 0.6 * alpha)

            msg_p = Double()
            msg_p.data = gmb_theta
            pub_gimbal_pitch.publish(msg_p)

        # camera pose in world (strictly at drone nose, oriented by pitch gimbal angle)
        p_cam_world = p_nose_world

        if gimbal_enabled:
            cp, sp = math.cos(gmb_theta), math.sin(gmb_theta)
            R_rel = np.array([
                [ cp,  0.,  sp],
                [ 0.,  1.,  0.],
                [-sp,  0.,  cp]
            ])
            R_cam = R_int @ R_rel
        else:
            R_cam = R_int

        # target relative to camera, in gz-camera frame
        d_world = p_tgt - p_cam_world
        d_cam_gz = R_cam_gz_body @ (R_cam.T @ d_world)

        # convert to camera optical frame
        d_opt = GZ_TO_OPT @ d_cam_gz  # [x_right, y_down, z_forward]

        range_m = float(np.linalg.norm(d_opt))

        # Adaptive Camera FPS based on distance:
        # Distance > 20m: 50 FPS (1920x1200 mode)
        # Distance <= 20m: 80 FPS (960x600 mode)
        if range_m <= 20.0 and not current_mode_near:
            current_mode_near = True
            current_rate = rate_near
            dt = 1.0 / current_rate
            print(f"[sim_detector] Range {range_m:.1f} m <= 20m -> Switched to 960x600 @ 80 FPS (terminal high-speed mode)", flush=True)
        elif range_m > 22.0 and current_mode_near:
            current_mode_near = False
            current_rate = rate_far
            dt = 1.0 / current_rate
            print(f"[sim_detector] Range {range_m:.1f} m > 20m -> Switched to 1920x1200 @ 50 FPS (far acquisition mode)", flush=True)

        # check: target must be in front of the camera and within range
        if d_opt[2] <= 0 or range_m > a.max_range or range_m < 0.1:
            last_visible = False
            pkt = pack_packet(frame_id, 0, 0, 0,
                              [0, 0, 1], 0, 0, [1, 0, 0, 0], [0, 0, 0, 0])
            sock.sendto(pkt, dest)
            msg_det = Double_V()
            msg_det.data.extend([0.0, 0.0, 0.0, 0.0, float(range_m), 0.0])
            pub_detection.publish(msg_det)
            continue

        # line of sight: unit vector camera → target in optical frame
        los = d_opt / range_m

        # check FOV: project the centre of the target
        u_centre = fx * los[0] / los[2] + cx
        v_centre = fy * los[1] / los[2] + cy

        # generous FOV check (target centre within 1.2× the image)
        margin = 1.2
        in_fov = (-width * (margin - 1) / 2 < u_centre < width * margin
                  and -height * (margin - 1) / 2 < v_centre < height * margin)

        if not in_fov:
            last_visible = False
            pkt = pack_packet(frame_id, 0, 0, 0,
                              [0, 0, 1], 0, 0, [1, 0, 0, 0], [0, 0, 0, 0])
            sock.sendto(pkt, dest)
            msg_det = Double_V()
            msg_det.data.extend([0.0, 0.0, 0.0, 0.0, float(range_m), 0.0])
            pub_detection.publish(msg_det)
            continue

        # --- target is visible ---

        # target attitude in camera optical frame
        # R_tgt is world→target-body (gz: x forward, y left, z up = FLU)
        # We want target FRD (x nose, y right wing, z down) in optical frame
        # Talon body in gz: x nose, y left, z up (FLU)
        # FLU → FRD: flip y and z
        R_flu_to_frd = np.diag([1.0, -1.0, -1.0])
        R_tgt_frd = R_tgt @ R_flu_to_frd  # columns of R_tgt_frd are FRD axes in world

        # camera optical frame axes in world:
        # R_cam_opt_world takes world vectors into optical frame
        R_cam_opt_world = GZ_TO_OPT @ R_cam_gz_body @ R_cam.T

        # target attitude in camera optical frame
        R_target_in_cam = R_cam_opt_world @ R_tgt_frd
        q = rot_to_quat(R_target_in_cam)

        # bounding box from projected vertices
        bbox = [0.0, 0.0, 0.0, 0.0]
        visible = False
        if talon_pts is not None:
            # transform Talon vertices relative to camera, then into camera optical frame
            pts_world = (R_tgt @ talon_pts.T).T + p_tgt
            pts_cam = (R_cam_opt_world @ (pts_world - p_cam_world).T).T
            # project only points in front of the camera
            in_front = pts_cam[:, 2] > 0.1
            if np.any(in_front):
                pts_f = pts_cam[in_front]
                u_raw = fx * pts_f[:, 0] / pts_f[:, 2] + cx
                v_raw = fy * pts_f[:, 1] / pts_f[:, 2] + cy
                # Unclipped bounding box of the full target
                u_raw_min, u_raw_max = float(u_raw.min()), float(u_raw.max())
                v_raw_min, v_raw_max = float(v_raw.min()), float(v_raw.max())
                bw_raw = u_raw_max - u_raw_min
                bh_raw = v_raw_max - v_raw_min
                raw_area = bw_raw * bh_raw

                # Clipped bounding box within the image frame [0, width] x [0, height]
                u_clip_min = max(0.0, u_raw_min)
                u_clip_max = min(float(width), u_raw_max)
                v_clip_min = max(0.0, v_raw_min)
                v_clip_max = min(float(height), v_raw_max)
                bw_clip = max(0.0, u_clip_max - u_clip_min)
                bh_clip = max(0.0, v_clip_max - v_clip_min)
                vis_area = bw_clip * bh_clip

                # Rule: Detection is lost if more than 1/3 of the target (bbox)
                # is outside the image (i.e. at least 2/3 (66.7%) of bbox area remains inside).
                vis_ratio = (vis_area / raw_area) if raw_area > 0 else 0.0

                if bw_clip > 0.5 and bh_clip > 0.5 and vis_ratio >= (2.0 / 3.0):
                    visible = True
                    bbox = [
                        float((u_clip_min + u_clip_max) / 2.0 / width),   # cx normalised [0, 1]
                        float((v_clip_min + v_clip_max) / 2.0 / height),  # cy normalised [0, 1]
                        float(bw_clip / width),                           # w normalised
                        float(bh_clip / height),                          # h normalised
                    ]
                    last_visible = True
                    last_bbox = bbox
                    last_seen_time = t_sim

        if not visible:
            last_visible = False
            pkt = pack_packet(frame_id, 0, 0, 0,
                              [0, 0, 1], 0, 0, [1, 0, 0, 0], [0, 0, 0, 0])
            sock.sendto(pkt, dest)
            msg_det = Double_V()
            msg_det.data.extend([0.0, 0.0, 0.0, 0.0, float(range_m), 0.0])
            pub_detection.publish(msg_det)
            continue

        if gimbal_enabled:
            # Line-of-sight in vehicle body FRD frame
            d_frd = np.array([d_tgt_body[0], -d_tgt_body[1], -d_tgt_body[2]])
            los_out = (d_frd / dist_tgt).tolist()
            # Target attitude in vehicle body FRD frame
            R_body_frd = R_int @ R_flu_to_frd
            R_target_in_body = R_body_frd.T @ R_tgt_frd
            q_out = list(rot_to_quat(R_target_in_body))
            flags = FLAG_DETECTED | FLAG_RANGE | FLAG_ATTITUDE | FLAG_BODY_LOS
        else:
            los_out = [float(los[0]), float(los[1]), float(los[2])]
            q_out = [float(q[0]), float(q[1]), float(q[2]), float(q[3])]
            flags = FLAG_DETECTED | FLAG_RANGE | FLAG_ATTITUDE

        confidence = 255  # perfect detection
        range_sigma = 0.1  # negligible uncertainty for ground truth

        pkt = pack_packet(
            frame_id, 0, flags, confidence,
            los_out,
            float(range_m), range_sigma,
            q_out,
            bbox
        )
        sock.sendto(pkt, dest)
        msg_det = Double_V()
        msg_det.data.extend([bbox[0], bbox[1], bbox[2], bbox[3], float(range_m), 1.0])
        pub_detection.publish(msg_det)

        # periodic status
        if t_sim - last_print >= 5.0:
            print(f"t={t_sim:7.1f}  range {range_m:6.1f} m  "
                  f"los [{los[0]:+.2f} {los[1]:+.2f} {los[2]:+.2f}]  "
                  f"bbox [{bbox[0]:.2f} {bbox[1]:.2f} {bbox[2]:.3f} {bbox[3]:.3f}]  "
                  f"frames {frame_id}", flush=True)
            last_print = t_sim


if __name__ == "__main__":
    main()
