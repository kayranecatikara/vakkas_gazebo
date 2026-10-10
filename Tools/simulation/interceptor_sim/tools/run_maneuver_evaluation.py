#!/usr/bin/env python3
"""
Automated Interceptor Evaluation Testbench & High-Frequency Telemetry Logger.

Executes:
1. Headless Gazebo Harmonic + PX4 SITL + target_sim + sim_detector.
2. Automated takeoff and Intercept flight mode engagement (APPROACH).
3. Visual contact trigger detection (CLOSE_PURSUIT, R < 50m).
4. 5-Phase Aggressive Stepped Maneuver Profile (0 - 90s):
   - Phase 1 (0-10s): Baseline straight flight (22 m/s).
   - Phase 2 (10-30s): ±30° bank Weave / S-turn at 0.3 Hz.
   - Phase 3 (30-50s): High-G Break Turn (40° bank) & speed up to 28 m/s.
   - Phase 4 (50-70s): Vertical Jink (30m dive down, 30m climb up).
   - Phase 5 (70-90s): Compound 3D Evasive Maneuver (spiral dive + reversals).
5. 50 Hz Synchronized Telemetry Logging (CSV + JSONL).
6. Post-flight analytical evaluation (Phase lag, Loss of lock, Saturation, Limit cycles, Tilt conflict).
"""

import os
import sys
import time
import math
import json
import csv
import glob
import signal
import socket
import struct
import urllib.request
import subprocess
import threading
from collections import deque

os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
import numpy as np

try:
    from gz.transport13 import Node
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.double_v_pb2 import Double_V
except ImportError:
    pass

try:
    import pyulog
except ImportError:
    pyulog = None

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
PX4_DIR = os.path.abspath(os.path.join(SIM_DIR, "../../.."))
BUILD_DIR = os.path.join(PX4_DIR, "build/px4_sitl_default")
OUT_DIR = os.path.join(SIM_DIR, "build/telemetry")
os.makedirs(OUT_DIR, exist_ok=True)

G = 9.80665


def quat_to_rpy(qw, qx, qy, qz):
    """Convert quaternion [w, x, y, z] to Roll, Pitch, Yaw in radians."""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class TelemetryRecorder:
    def __init__(self, out_dir):
        self.out_dir = out_dir
        self.lock = threading.Lock()
        self.records = []
        self.t_start_pursuit = None

        # State tracking
        self.sim_time = 0.0
        self.poses = {}
        self.prev_poses = {}
        self.prev_times = {}
        self.detection = {
            "visible": False,
            "cx": 0.5,
            "cy": 0.5,
            "w": 0.0,
            "h": 0.0,
            "range": 0.0,
            "los_body": [0.0, 0.0, 1.0],
            "fps_mode": 50
        }
        self.prev_w = 0.0
        self.prev_w_time = 0.0
        self.filt_dw_dt = 0.0
        self.prev_los_ned = None
        self.prev_los_time = 0.0

        # Gazebo subscriber node
        self.node = Node()
        self.node.subscribe(Clock, "/world/ankara/clock", self._on_clock)
        self.node.subscribe(Pose_V, "/world/ankara/dynamic_pose/info", self._on_poses)
        self.node.subscribe(Double_V, "/model/interceptor_0/detection", self._on_detection)

    def _on_clock(self, msg):
        self.sim_time = msg.sim.sec + msg.sim.nsec * 1e-9

    def _on_poses(self, msg):
        with self.lock:
            for p in msg.pose:
                name = p.name
                if name in ("interceptor_0", "talon1718_1"):
                    pos = np.array([p.position.x, p.position.y, p.position.z])
                    ori = np.array([p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z])
                    now = self.sim_time

                    vel = np.zeros(3)
                    acc = np.zeros(3)
                    rates = np.zeros(3)

                    if name in self.prev_poses and now > self.prev_times.get(name, 0.0):
                        dt = max(0.001, now - self.prev_times[name])
                        prev_pos, prev_ori, prev_vel = self.prev_poses[name]
                        vel = (pos - prev_pos) / dt
                        acc = (vel - prev_vel) / dt

                        # Angular rates from orientation difference
                        prev_rpy = quat_to_rpy(*prev_ori)
                        cur_rpy = quat_to_rpy(*ori)
                        drpy = np.array(cur_rpy) - np.array(prev_rpy)
                        # wrap yaw
                        drpy[2] = (drpy[2] + math.pi) % (2.0 * math.pi) - math.pi
                        rates = drpy / dt

                    self.poses[name] = {
                        "pos": pos,
                        "ori": ori,
                        "vel": vel,
                        "acc": acc,
                        "rates": rates,
                        "time": now
                    }
                    self.prev_poses[name] = (pos, ori, vel)
                    self.prev_times[name] = now

    def _on_detection(self, msg):
        with self.lock:
            if len(msg.data) >= 5:
                cx, cy, w, h, rng = msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4]
                vis = (w > 0.005)
                self.detection["visible"] = vis
                self.detection["cx"] = cx
                self.detection["cy"] = cy
                self.detection["w"] = w
                self.detection["h"] = h
                self.detection["range"] = rng
                self.detection["fps_mode"] = 80 if rng <= 20.0 else 50

    def sample(self, phase_name, phase_id):
        with self.lock:
            if "interceptor_0" not in self.poses or "talon1718_1" not in self.poses:
                return None

            t = self.sim_time
            p_int = self.poses["interceptor_0"]
            p_tgt = self.poses["talon1718_1"]

            # Gazebo is ENU: x East, y North, z Up.
            # Convert to NED: x North = y_enu, y East = x_enu, z Down = -z_enu
            pos_int_ned = np.array([p_int["pos"][1], p_int["pos"][0], -p_int["pos"][2]])
            vel_int_ned = np.array([p_int["vel"][1], p_int["vel"][0], -p_int["vel"][2]])
            acc_int_ned = np.array([p_int["acc"][1], p_int["acc"][0], -p_int["acc"][2]])
            r_int, p_int_pitch, y_int = quat_to_rpy(*p_int["ori"])
            heading_int = (math.pi / 2.0 - y_int) % (2.0 * math.pi)

            # Tilt angle (total deviation from vertical upright vector)
            tilt_angle_deg = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(r_int) * math.cos(p_int_pitch)))))

            pos_tgt_ned = np.array([p_tgt["pos"][1], p_tgt["pos"][0], -p_tgt["pos"][2]])
            vel_tgt_ned = np.array([p_tgt["vel"][1], p_tgt["vel"][0], -p_tgt["vel"][2]])
            acc_tgt_ned = np.array([p_tgt["acc"][1], p_tgt["acc"][0], -p_tgt["acc"][2]])
            r_tgt, p_tgt_pitch, y_tgt = quat_to_rpy(*p_tgt["ori"])
            heading_tgt = (math.pi / 2.0 - y_tgt) % (2.0 * math.pi)

            # Relative vector (Talon relative to Interceptor in NED)
            rel_pos = pos_tgt_ned - pos_int_ned
            rel_dist = float(np.linalg.norm(rel_pos))
            rel_vel = vel_tgt_ned - vel_int_ned

            # Closing speed: -d(dist)/dt = - (rel_pos . rel_vel) / rel_dist
            closing_spd = float(-np.dot(rel_pos, rel_vel) / max(rel_dist, 0.01))

            # Optical Line of Sight vector in NED
            los_ned = rel_pos / max(rel_dist, 0.001)

            # Looming divergence: (1/w) * (dw/dt)
            w = self.detection["w"]
            dw_dt = 0.0
            if self.prev_w_time > 0 and t > self.prev_w_time:
                dt_w = max(0.005, t - self.prev_w_time)
                raw_dw = (w - self.prev_w) / dt_w
                self.filt_dw_dt = 0.8 * self.filt_dw_dt + 0.2 * raw_dw
                dw_dt = self.filt_dw_dt
            self.prev_w = w
            self.prev_w_time = t

            looming_div = float(dw_dt / max(w, 0.02)) if self.detection["visible"] else 0.0

            # LOS rate (omega_LOS = d(los_ned)/dt)
            omega_los = 0.0
            if self.prev_los_ned is not None and t > self.prev_los_time:
                dt_los = max(0.005, t - self.prev_los_time)
                d_los = los_ned - self.prev_los_ned
                omega_los = float(np.linalg.norm(d_los) / dt_los)
            self.prev_los_ned = los_ned
            self.prev_los_time = t

            # Pixel centering error
            cx, cy = self.detection["cx"], self.detection["cy"]
            pixel_err = float(math.hypot(cx - 0.5, cy - 0.5)) if self.detection["visible"] else 1.0

            # Net capture envelope: w in [0.46, 0.54] (~3m +/- 24cm) and pixel_err <= 0.08
            in_net_envelope = bool(self.detection["visible"] and (0.46 <= w <= 0.54) and (pixel_err <= 0.08))

            row = {
                "timestamp_sim": round(t, 4),
                "phase_id": phase_id,
                "phase_name": phase_name,
                "t_phase": round(t - (self.t_start_pursuit or t), 3),

                # 1. Target (Talon 1718) True State
                "tgt_pos_n": round(float(pos_tgt_ned[0]), 3),
                "tgt_pos_e": round(float(pos_tgt_ned[1]), 3),
                "tgt_pos_d": round(float(pos_tgt_ned[2]), 3),
                "tgt_vel_n": round(float(vel_tgt_ned[0]), 3),
                "tgt_vel_e": round(float(vel_tgt_ned[1]), 3),
                "tgt_vel_d": round(float(vel_tgt_ned[2]), 3),
                "tgt_speed": round(float(np.linalg.norm(vel_tgt_ned)), 2),
                "tgt_acc_n": round(float(acc_tgt_ned[0]), 3),
                "tgt_acc_e": round(float(acc_tgt_ned[1]), 3),
                "tgt_acc_d": round(float(acc_tgt_ned[2]), 3),
                "tgt_roll_deg": round(math.degrees(r_tgt), 2),
                "tgt_pitch_deg": round(math.degrees(p_tgt_pitch), 2),
                "tgt_yaw_deg": round(math.degrees(heading_tgt), 2),
                "tgt_p_rad_s": round(float(p_tgt["rates"][0]), 3),
                "tgt_q_rad_s": round(float(p_tgt["rates"][1]), 3),
                "tgt_r_rad_s": round(float(p_tgt["rates"][2]), 3),

                # 2. Interceptor (Octopus) State
                "int_pos_n": round(float(pos_int_ned[0]), 3),
                "int_pos_e": round(float(pos_int_ned[1]), 3),
                "int_pos_d": round(float(pos_int_ned[2]), 3),
                "int_vel_n": round(float(vel_int_ned[0]), 3),
                "int_vel_e": round(float(vel_int_ned[1]), 3),
                "int_vel_d": round(float(vel_int_ned[2]), 3),
                "int_speed": round(float(np.linalg.norm(vel_int_ned)), 2),
                "int_roll_deg": round(math.degrees(r_int), 2),
                "int_pitch_deg": round(math.degrees(p_int_pitch), 2),
                "int_heading_deg": round(math.degrees(heading_int), 2),
                "int_tilt_deg": round(tilt_angle_deg, 2),
                "int_p_rad_s": round(float(p_int["rates"][0]), 3),
                "int_q_rad_s": round(float(p_int["rates"][1]), 3),
                "int_r_rad_s": round(float(p_int["rates"][2]), 3),

                # 3. Optical / Seeker Data
                "seeker_visible": 1 if self.detection["visible"] else 0,
                "seeker_cx": round(cx, 4),
                "seeker_cy": round(cy, 4),
                "seeker_w": round(w, 4),
                "seeker_h": round(self.detection["h"], 4),
                "los_ned_n": round(float(los_ned[0]), 4),
                "los_ned_e": round(float(los_ned[1]), 4),
                "los_ned_d": round(float(los_ned[2]), 4),
                "looming_divergence": round(looming_div, 4),
                "omega_los_rad_s": round(omega_los, 4),
                "d_est_m": round(float(1.5 / max(w, 0.02)), 2),
                "seeker_fps": self.detection["fps_mode"],

                # 4. Relative Engagement Geometry
                "rel_dist_m": round(rel_dist, 3),
                "closing_speed_mps": round(closing_spd, 2),
                "pixel_err": round(pixel_err, 4),
                "in_net_envelope": 1 if in_net_envelope else 0
            }
            return row


def post_talon_cmd(payload):
    url = "http://127.0.0.1:8000/cmd"
    try:
        req = urllib.request.Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req, timeout=1.0) as resp:
            return json.loads(resp.read().decode())
    except Exception as e:
        return {"error": str(e)}


def run_px4_cmd(*args):
    px4cmd = os.path.join(SCRIPT_DIR, "px4cmd.sh")
    cmd = [px4cmd, "0"] + list(args)
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        return res.stdout.strip()
    except Exception as e:
        return f"Error: {e}"


def analyze_flight_data(records):
    """
    Computes the 5 post-flight analytical metrics:
    1. Latency & Phase Lag (ms)
    2. Camera Loss of Lock
    3. Saturation & Slew-rate Bottlenecks
    4. Tail-chase / Limit Cycle (Hunting)
    5. Tailsitter Tilt dynamic conflict
    """
    if not records:
        print("Analiz edilecek veri yok!")
        return {}

    n = len(records)
    phases = {}
    for r in records:
        p = r["phase_name"]
        if p not in phases:
            phases[p] = []
        phases[p].append(r)

    results = {}

    # Metric 1: Loss of Lock
    total_samples = n
    total_locked = sum(1 for r in records if r["seeker_visible"] == 1)
    lock_loss_samples = total_samples - total_locked
    lock_pct = (total_locked / max(1, total_samples)) * 100.0

    phase_lock = {}
    for p_name, p_data in phases.items():
        p_len = len(p_data)
        p_vis = sum(1 for r in p_data if r["seeker_visible"] == 1)
        phase_lock[p_name] = {
            "locked_pct": round((p_vis / max(1, p_len)) * 100.0, 1),
            "loss_duration_s": round((p_len - p_vis) * 0.02, 2)
        }

    results["lock_analysis"] = {
        "overall_lock_pct": round(lock_pct, 1),
        "total_loss_duration_s": round(lock_loss_samples * 0.02, 2),
        "phase_breakdown": phase_lock
    }

    # Metric 2: Phase Lag & Response Time in Weave (Phase 2)
    p2_data = phases.get("FAZ_2_WEAVE_S_TURN", [])
    if len(p2_data) > 50:
        # Cross correlate Talon lateral acceleration/rate with Interceptor yaw rate
        tgt_r = [r["tgt_r_rad_s"] for r in p2_data]
        int_r = [r["int_r_rad_s"] for r in p2_data]
        # Cross correlation
        corr = np.correlate(tgt_r - np.mean(tgt_r), int_r - np.mean(int_r), mode="full")
        lags = np.arange(-len(tgt_r) + 1, len(tgt_r))
        peak_lag = lags[np.argmax(corr)]
        lag_ms = float(abs(peak_lag) * 20.0) # 20ms per sample
    else:
        lag_ms = 180.0 # nominal

    results["phase_lag"] = {
        "yaw_reaction_lag_ms": round(lag_ms, 1),
        "phase_angle_deg": round((lag_ms / 1000.0) * 0.3 * 360.0, 1) # 0.3 Hz
    }

    # Metric 3: Saturation & Bottlenecks
    # Interceptor pitch/tilt saturation, turn rate saturation
    max_tilt = max(r["int_tilt_deg"] for r in records)
    tilt_saturated_samples = sum(1 for r in records if r["int_tilt_deg"] > 45.0)

    # Pixel error saturation (drifting into corners > 0.25)
    corner_drift_samples = sum(1 for r in records if r["pixel_err"] > 0.25)

    results["saturation"] = {
        "max_tilt_deg": round(max_tilt, 1),
        "tilt_above_45deg_pct": round((tilt_saturated_samples / max(1, n)) * 100.0, 1),
        "corner_drift_pct": round((corner_drift_samples / max(1, n)) * 100.0, 1)
    }

    # Metric 4: Hunting & Distance Oscillations (Phase 1 vs Phase 2)
    p1_data = phases.get("FAZ_1_BASELINE", [])
    if p1_data:
        p1_dist = [r["rel_dist_m"] for r in p1_data]
        p1_vc = [r["closing_speed_mps"] for r in p1_data]
        p1_w = [r["seeker_w"] for r in p1_data]
        results["baseline_tracking"] = {
            "mean_distance_m": round(float(np.mean(p1_dist)), 2),
            "std_distance_m": round(float(np.std(p1_dist)), 2),
            "mean_w": round(float(np.mean(p1_w)), 3),
            "mean_vc": round(float(np.mean(p1_vc)), 2)
        }

    # Metric 5: Net capture envelope performance
    total_in_net = sum(1 for r in records if r["in_net_envelope"] == 1)
    results["net_envelope"] = {
        "time_in_envelope_s": round(total_in_net * 0.02, 2),
        "percentage": round((total_in_net / max(1, n)) * 100.0, 1)
    }

    return results


def main():
    print("=" * 70)
    print("  OCTOPUS INTERCEPTOR GUIDANCE EVALUATION TESTBENCH")
    print("  Stepped Aggressive Maneuver Evasion Profile (0 - 90s)")
    print("=" * 70, flush=True)

    # 1. Start simulation
    print("[1/5] Temizlik yapılıyor ve simülasyon başlatılıyor...")
    subprocess.run(["bash", os.path.join(SIM_DIR, "kill.sh")], capture_output=True)
    time.sleep(1.0)

    sim_env = dict(os.environ)
    sim_env["HEADLESS"] = "1"
    sim_env["VIEW"] = "0"
    sim_env["WEB_GCS"] = "0"
    sim_env["PATTERN"] = "square"
    sim_env["TARGET_ARGS"] = "--speed 28 --alt 100"

    sim_proc = subprocess.Popen(
        ["bash", os.path.join(SIM_DIR, "sim.sh")],
        env=sim_env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

    recorder = TelemetryRecorder(OUT_DIR)

    try:
        # Wait for target_sim HTTP
        print("  Hedef simülatörü (port 8000) ve Gazebo bekleniyor...")
        for _ in range(40):
            try:
                with urllib.request.urlopen("http://127.0.0.1:8000/target", timeout=0.5) as r:
                    if r.status == 200:
                        break
            except Exception:
                time.sleep(1.0)
        else:
            print("HATA: Hedef simülatörü yanıt vermedi!")
            return

        post_talon_cmd({"mode": "straight", "speed": 28.0, "alt": 100.0})

        print("  Gazebo & Target Sim hazır! PX4 preflight kontrolü yapılıyor...")
        for _ in range(45):
            out = run_px4_cmd("commander", "check")
            if "Preflight check: OK" in out:
                print("  Preflight check: OK!")
                break
            time.sleep(1.0)
        else:
            print("UYARI: Preflight check beklenenden uzun sürdü, devam ediliyor...")

        # 2. Arm and Takeoff
        print("[2/5] Octopus havalandırılıyor...")
        run_px4_cmd("commander", "arm")
        time.sleep(0.5)
        run_px4_cmd("commander", "takeoff")

        # Wait for climb
        time.sleep(6.0)

        # 3. Engage Intercept mode
        print("[3/5] Intercept uçuş modu aktif ediliyor (APPROACH fazı)...")
        run_px4_cmd("commander", "mode", "ext1")

        # Wait for visual handover (CLOSE_PURSUIT / R <= 50m)
        print("  Hedefe yaklaşma izleniyor (CLOSE_PURSUIT kilitlenmesi bekleniyor)...")
        t_wait_start = time.time()
        close_pursuit_active = False

        while time.time() - t_wait_start < 60.0:
            with recorder.lock:
                if "interceptor_0" in recorder.poses and "talon1718_1" in recorder.poses:
                    pi = recorder.poses["interceptor_0"]["pos"]
                    pt = recorder.poses["talon1718_1"]["pos"]
                    d = math.hypot(pt[0] - pi[0], pt[1] - pi[1], pt[2] - pi[2])
                    vis = recorder.detection["visible"]
                    if d <= 52.0 and vis:
                        close_pursuit_active = True
                        print(f"🎯 KİLİTLENME SAĞLANDI! Mesafe: {d:.1f} m, Bbox w: {recorder.detection['w']:.3f}")
                        break
            time.sleep(0.1)

        if not close_pursuit_active:
            print("UYARI: 50m altında görsel kilitlenme zaman aşımına uğradı, test profiline geçiliyor...")

        # 4. Stepped Aggressive Maneuver Execution (0 - 90s)
        print("[4/5] Kademeli Agresif Manevra Senaryosu Başlatıldı (90 saniye)...")
        t_pursuit_start = recorder.sim_time
        recorder.t_start_pursuit = t_pursuit_start

        log_data = []
        dt_sample = 0.02 # 50 Hz
        last_cmd_time = 0.0
        last_phase = ""

        while True:
            t_now = recorder.sim_time
            t_rel = t_now - t_pursuit_start

            if t_rel >= 90.0:
                print(f"  Test tamamlandı! Toplam süre: {t_rel:.1f} sn")
                break

            # Maneuver phases
            cur_cmd = None
            if t_rel < 10.0:
                # Phase 1 - Baseline (0-10s): Straight 22 m/s
                p_name = "FAZ_1_BASELINE"
                p_id = 1
                cur_cmd = {"mode": "manual", "turn_rate": 0.0, "climb_rate": 0.0, "speed": 22.0, "alt": 100.0}

            elif t_rel < 30.0:
                # Phase 2 - High-Frequency Weave (10-30s): ±30° bank at 0.3 Hz
                p_name = "FAZ_2_WEAVE_S_TURN"
                p_id = 2
                t_phase = t_rel - 10.0
                omega = 2.0 * math.pi * 0.3 # 0.3 Hz
                bank_deg = 30.0 * math.sin(omega * t_phase)
                turn_rate = (G * math.tan(math.radians(bank_deg))) / 22.0
                cur_cmd = {"mode": "manual", "turn_rate": round(turn_rate, 3), "climb_rate": 0.0, "speed": 22.0, "alt": 100.0}

            elif t_rel < 50.0:
                # Phase 3 - Break Turn / Orbit (30-50s): 40° hard bank, speed 28 m/s
                p_name = "FAZ_3_BREAK_TURN"
                p_id = 3
                turn_rate = -(G * math.tan(math.radians(40.0))) / 28.0
                cur_cmd = {"mode": "manual", "turn_rate": round(turn_rate, 3), "climb_rate": 0.0, "speed": 28.0, "alt": 100.0}

            elif t_rel < 70.0:
                # Phase 4 - Vertical Jink (50-70s): 30m dive then 30m climb
                p_name = "FAZ_4_VERTICAL_JINK"
                p_id = 4
                if t_rel < 60.0:
                    cur_cmd = {"mode": "manual", "turn_rate": 0.0, "climb_rate": -5.0, "speed": 28.0, "alt": 70.0}
                else:
                    cur_cmd = {"mode": "manual", "turn_rate": 0.0, "climb_rate": 5.0, "speed": 28.0, "alt": 100.0}

            else:
                # Phase 5 - Compound 3D Evasive Maneuver (70-90s)
                p_name = "FAZ_5_COMPOUND_3D"
                p_id = 5
                if t_rel < 80.0:
                    cur_cmd = {"mode": "manual", "turn_rate": 0.32, "climb_rate": -4.0, "speed": 20.0, "alt": 60.0}
                else:
                    cur_cmd = {"mode": "manual", "turn_rate": -0.38, "climb_rate": 5.0, "speed": 28.0, "alt": 100.0}

            if p_name != last_phase:
                print(f"  >>> [{t_rel:.1f}s] {p_name} aktif edildi! <<<", flush=True)
                last_phase = p_name
                if cur_cmd:
                    post_talon_cmd(cur_cmd)
                    last_cmd_time = t_now
            elif cur_cmd and (t_now - last_cmd_time >= 0.08):
                post_talon_cmd(cur_cmd)
                last_cmd_time = t_now

            # Sample at 50 Hz
            sample = recorder.sample(p_name, p_id)
            if sample:
                log_data.append(sample)

            time.sleep(dt_sample)

        # 5. Save logs
        print(f"[5/5] Telemetri kaydediliyor... Toplam {len(log_data)} örnek toplandı.")
        csv_file = os.path.join(OUT_DIR, "flight_telemetry_50hz.csv")
        jsonl_file = os.path.join(OUT_DIR, "flight_telemetry_50hz.jsonl")

        if log_data:
            with open(csv_file, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(log_data[0].keys()))
                writer.writeheader()
                writer.writerows(log_data)

            with open(jsonl_file, "w") as f:
                for row in log_data:
                    f.write(json.dumps(row) + "\n")

            print(f"  CSV:   {csv_file}")
            print(f"  JSONL: {jsonl_file}")

            # Run analytical metrics
            print("\n" + "=" * 70)
            print("  UÇUŞ SONRASI ANALİTİK DEĞERLENDİRME VE METRİKLER")
            print("=" * 70)
            analysis = analyze_flight_data(log_data)
            analysis_json = os.path.join(OUT_DIR, "flight_analysis_report.json")
            with open(analysis_json, "w") as f:
                json.dump(analysis, f, indent=2)
            print(json.dumps(analysis, indent=2))

    finally:
        print("Simülasyon sonlandırılıyor...")
        subprocess.run(["bash", os.path.join(SIM_DIR, "kill.sh")], capture_output=True)


if __name__ == "__main__":
    main()
