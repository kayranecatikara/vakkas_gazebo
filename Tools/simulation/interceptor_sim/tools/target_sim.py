#!/usr/bin/env python3
"""
Kinematic Talon target (no PX4) and its position telemetry.

The target model is moved with Gazebo's VelocityControl system (body frame
velocity commands, applied every physics step) along a scripted path, in sim
time: take-off roll along the runway, climb, then a square circuit with
rounded corners, forever. Position and attitude errors are corrected from the
ground truth pose, roll follows the turn rate (coordinated turn) and pitch the
climb angle.

Telemetry of the target, at --rate Hz (default 2):
- HTTP JSON:  GET http://localhost:8000/target
- ADS-B:      ADSB_VEHICLE to the interceptor PX4 (udp 14580, its API link); PX4 forwards
              it to QGroundControl, which draws the target on the map

The first side of the square is the runway extension (world origin = runway
start, see tools/build_world.py), the square extends to the left of it.

    python3 tools/target_sim.py [--alt 100] [--speed 25] [--side 2000] [--http-port 8000]
"""

import argparse
import json
import math
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
import re
import threading
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node
from pymavlink import mavutil

SIM_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
G = 9.81
mav = mavutil.mavlink


# ------------------------------------------------------------------ geometry ---

def rot_rpy(roll, pitch, yaw):
	cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
	return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
			 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
			 [-sp, cp * sr, cp * cr]])


def quat_to_rot(w, x, y, z):
	return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
			 [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
			 [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def rot_log(R):
	"""Rotation vector of R."""
	angle = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))

	if angle < 1e-6:
		return np.zeros(3)

	return angle / (2 * math.sin(angle)) * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


class Path:
	"""2D path of lines and arcs (ENU), parameterised by arc length; closed loop after `loop_start`."""

	def __init__(self):
		self.segs = []  # (kind, length, data)

	def line(self, a, b):
		a, b = np.asarray(a, float), np.asarray(b, float)
		self.segs.append(("line", float(np.linalg.norm(b - a)), (a, b)))

	def arc(self, center, radius, a0, sweep):
		self.segs.append(("arc", abs(sweep) * radius, (np.asarray(center, float), radius, a0, sweep)))

	@property
	def length(self):
		return sum(s[1] for s in self.segs)

	def at(self, s):
		"""position, heading (ENU yaw), curvature (+ left turn)"""
		for kind, length, d in self.segs:
			if s <= length:
				break

			s -= length

		s = min(s, length)

		if kind == "line":
			a, b = d
			u = (b - a) / length
			return a + u * s, math.atan2(u[1], u[0]), 0.0

		c, r, a0, sweep = d
		a = a0 + math.copysign(s / r, sweep)
		p = c + r * np.array([math.cos(a), math.sin(a)])
		return p, a + math.copysign(math.pi / 2, sweep), math.copysign(1 / r, sweep)


def square_circuit(side, heading, radius):
	"""Square with rounded corners (left turns), first side from the origin along `heading`.
	The loop starts on the first side, `radius` after the origin (end of the last corner)."""
	u = np.array([math.cos(heading), math.sin(heading)])
	v = np.array([-u[1], u[0]])
	corners = [np.zeros(2), side * u, side * (u + v), side * v]
	path = Path()

	for i in range(4):
		c0, c1 = corners[i], corners[(i + 1) % 4]
		d = (c1 - c0) / side
		n = np.array([-d[1], d[0]])  # left
		path.line(c0 + d * radius, c1 - d * radius)
		path.arc(c1 - d * radius + n * radius, radius, math.atan2(-n[1], -n[0]), math.pi / 2)

	return path


# ----------------------------------------------------------------- profile ---

class Profile:
	"""Speed and altitude along the path: take-off roll, climb, cruise; continuous weave, square, or dynamic maneuvers."""

	def __init__(self, a):
		self.a = a
		self.radius = a.speed ** 2 / (G * math.tan(math.radians(a.bank)))
		self.yaw0 = math.radians(90 - a.heading)
		self.u = np.array([math.cos(self.yaw0), math.sin(self.yaw0)])
		self.n = np.array([-self.u[1], self.u[0]])  # left normal

		self.path = square_circuit(a.side, self.yaw0, self.radius)
		self.lap = self.path.length
		self.wave_amp = getattr(a, "weave_amp", 30.0)
		self.wave_len = getattr(a, "weave_wavelength", 400.0)
		self.k_wave = 2.0 * math.pi / self.wave_len

		self.s_roll = a.speed ** 2 / (2 * a.accel)
		self.gamma = math.radians(a.climb_angle)
		self.s_climb_end = self.s_roll + (a.alt - a.z0) / math.tan(self.gamma)

		# Dynamic scenario control state (thread-safe, updated via HTTP POST /cmd)
		self.lock = threading.Lock()
		self.mode = getattr(a, "pattern", "square")
		self.pattern = self.mode
		self.target_speed = float(a.speed)
		self.target_alt = float(a.alt)
		self.manual_turn_rate = 0.0   # rad/s (+ left, - right)
		self.manual_climb_rate = 0.0  # m/s (+ up, - down)
		self.circle_dir = 1.0         # +1 left, -1 right

		# Kinematic state for dynamic maneuvers
		self.dyn_initialized = False
		self.p_dyn = np.array([0.0, 0.0, a.z0], dtype=float)
		self.yaw_dyn = self.yaw0
		self.v_dyn = a.speed
		self.last_t = None
		self.weave_phase = 0.0

	def apply_cmd(self, cmd):
		with self.lock:
			if "mode" in cmd:
				new_mode = str(cmd["mode"]).lower().strip()
				if new_mode in ("square", "weave", "circle", "orbit", "straight", "dive", "climb", "manual"):
					if new_mode == "orbit":
						new_mode = "circle"
					self.mode = new_mode
			if "speed" in cmd:
				self.target_speed = float(np.clip(float(cmd["speed"]), 12.0, 42.0))
			if "alt" in cmd:
				self.target_alt = float(np.clip(float(cmd["alt"]), 25.0, 300.0))
			if "turn_rate" in cmd:
				self.manual_turn_rate = float(np.clip(float(cmd["turn_rate"]), -0.8, 0.8))
			if "climb_rate" in cmd:
				self.manual_climb_rate = float(np.clip(float(cmd["climb_rate"]), -10.0, 10.0))
			if "weave_amp" in cmd:
				self.wave_amp = float(np.clip(float(cmd["weave_amp"]), 10.0, 80.0))
			if "circle_dir" in cmd:
				self.circle_dir = 1.0 if float(cmd["circle_dir"]) >= 0 else -1.0

	def get_cmd_status(self):
		with self.lock:
			return {
				"mode": self.mode,
				"target_speed": self.target_speed,
				"target_alt": self.target_alt,
				"manual_turn_rate": self.manual_turn_rate,
				"manual_climb_rate": self.manual_climb_rate,
				"weave_amp": self.wave_amp,
				"circle_dir": self.circle_dir,
			}

	def speed_at(self, t):
		return min(self.a.speed, self.a.accel * t)

	def s_at(self, t):
		t_acc = self.a.speed / self.a.accel
		return 0.5 * self.a.accel * t * t if t < t_acc else self.s_roll + self.a.speed * (t - t_acc)

	def state(self, t):
		"""position (ENU), velocity, rotation, world angular velocity"""
		s = self.s_at(t)  # from the runway start
		v = self.speed_at(t)

		with self.lock:
			cur_mode = self.mode
			tgt_speed = self.target_speed
			tgt_alt = self.target_alt
			man_turn = self.manual_turn_rate
			man_climb = self.manual_climb_rate
			c_dir = self.circle_dir

		# 1. Takeoff roll & climb-out: always strictly follows runway trajectory
		if s < self.s_roll:
			p2, yaw, kappa = s * self.u, self.yaw0, 0.0
			z, gamma = self.a.z0, 0.0
			R = rot_rpy(0.0, 0.0, yaw)
			vel = v * np.array([math.cos(yaw), math.sin(yaw), 0.0])
			omega = np.zeros(3)
			self.last_t = t
			return np.array([p2[0], p2[1], z]), vel, R, omega

		# 2. Default initial square circuit (until user triggers dynamic scenario)
		if cur_mode == "square" and not self.dyn_initialized:
			s_loop = s - self.radius
			p2, yaw, kappa = self.path.at(s_loop % self.lap)
			z, gamma = (tgt_alt, 0.0) if s >= self.s_climb_end else (self.a.z0 + (s - self.s_roll) * math.tan(self.gamma), self.gamma)
			roll = -math.atan(v * v * kappa / G)
			pitch = -gamma
			R = rot_rpy(roll, pitch, yaw)
			vel = v * np.array([math.cos(gamma) * math.cos(yaw), math.cos(gamma) * math.sin(yaw), math.sin(gamma)])
			omega = np.array([0.0, 0.0, v * kappa])
			self.last_t = t
			return np.array([p2[0], p2[1], z]), vel, R, omega

		# 3. Active Dynamic Maneuver Loop
		dt = 0.02 if self.last_t is None else max(0.001, min(0.2, t - self.last_t))
		self.last_t = t

		if not self.dyn_initialized:
			s_loop = s - self.radius
			p2, yaw, _ = self.path.at(s_loop % self.lap)
			z_init = tgt_alt if s >= self.s_climb_end else (self.a.z0 + (s - self.s_roll) * math.tan(self.gamma))
			self.p_dyn = np.array([p2[0], p2[1], z_init], dtype=float)
			self.yaw_dyn = yaw
			self.v_dyn = v
			self.dyn_initialized = True

		# Speed transition
		acc_lim = getattr(self.a, "accel", 3.0) * dt
		self.v_dyn += float(np.clip(tgt_speed - self.v_dyn, -acc_lim, acc_lim))
		v_cur = self.v_dyn

		kappa = 0.0
		yaw_rate = 0.0
		climb_rate = 0.0

		if cur_mode == "square":
			s_loop = s - self.radius
			p2, yaw_sq, kappa_sq = self.path.at(s_loop % self.lap)
			self.p_dyn[0] += 0.04 * (p2[0] - self.p_dyn[0])
			self.p_dyn[1] += 0.04 * (p2[1] - self.p_dyn[1])
			self.yaw_dyn = yaw_sq
			yaw_rate = v_cur * kappa_sq
			alt_err = tgt_alt - self.p_dyn[2]
			climb_rate = float(np.clip(alt_err * 0.5, -4.0, 4.0))

		elif cur_mode == "weave":
			# Sinusoidal evasive S-turns
			self.weave_phase += 2.0 * math.pi * 0.15 * dt
			amp_rad = math.radians(26.0)
			yaw_rate = amp_rad * (2.0 * math.pi * 0.15) * math.cos(self.weave_phase)
			kappa = yaw_rate / max(v_cur, 1.0)
			alt_err = tgt_alt - self.p_dyn[2]
			climb_rate = float(np.clip(alt_err * 0.5, -4.0, 4.0))

		elif cur_mode == "circle":
			# Continuous coordinated turn orbit
			bank_rad = math.radians(getattr(self.a, "bank", 22.0))
			yaw_rate = c_dir * (G * math.tan(bank_rad)) / max(v_cur, 1.0)
			kappa = yaw_rate / max(v_cur, 1.0)
			alt_err = tgt_alt - self.p_dyn[2]
			climb_rate = float(np.clip(alt_err * 0.5, -4.0, 4.0))

		elif cur_mode == "straight":
			yaw_rate = 0.0
			kappa = 0.0
			alt_err = tgt_alt - self.p_dyn[2]
			climb_rate = float(np.clip(alt_err * 0.5, -4.0, 4.0))

		elif cur_mode == "dive":
			yaw_rate = 0.0
			kappa = 0.0
			dive_target = min(tgt_alt, 35.0)
			alt_err = dive_target - self.p_dyn[2]
			climb_rate = float(np.clip(alt_err * 0.8, -7.0, 0.0))

		elif cur_mode == "climb":
			yaw_rate = 0.0
			kappa = 0.0
			climb_target = max(tgt_alt, 150.0)
			alt_err = climb_target - self.p_dyn[2]
			climb_rate = float(np.clip(alt_err * 0.8, 0.0, 7.0))

		elif cur_mode == "manual":
			yaw_rate = man_turn
			kappa = yaw_rate / max(v_cur, 1.0)
			climb_rate = man_climb

		# Step kinematic integration
		self.yaw_dyn += yaw_rate * dt
		gamma = math.atan2(climb_rate, max(v_cur, 1.0))
		h_speed = v_cur * math.cos(gamma)

		self.p_dyn[0] += h_speed * math.cos(self.yaw_dyn) * dt
		self.p_dyn[1] += h_speed * math.sin(self.yaw_dyn) * dt
		self.p_dyn[2] = max(10.0, self.p_dyn[2] + climb_rate * dt)

		roll = -math.atan(v_cur * v_cur * kappa / G)
		pitch = -gamma
		R = rot_rpy(roll, pitch, self.yaw_dyn)
		vel = np.array([h_speed * math.cos(self.yaw_dyn), h_speed * math.sin(self.yaw_dyn), climb_rate])
		omega = np.array([0.0, 0.0, yaw_rate])

		return np.copy(self.p_dyn), vel, R, omega


# --------------------------------------------------------------- telemetry ---

class Telemetry:
	def __init__(self, lat0, lon0, alt0):
		self.lat0, self.lon0, self.alt0 = lat0, lon0, alt0
		self.lock = threading.Lock()
		self.data = {"valid": False}

	def update(self, t_sim, p, v, R, mode="square", tgt_speed=25.0, tgt_alt=100.0):
		lat = self.lat0 + p[1] / 111132.95
		lon = self.lon0 + p[0] / (111319.49 * math.cos(math.radians(self.lat0)))
		heading = (90 - math.degrees(math.atan2(v[1], v[0]))) % 360 if np.hypot(v[0], v[1]) > 0.5 else \
			(90 - math.degrees(math.atan2(R[1, 0], R[0, 0]))) % 360
		roll = math.degrees(math.atan2(R[2, 1], R[2, 2]))
		pitch = math.degrees(-math.asin(max(-1, min(1, R[2, 0]))))

		with self.lock:
			self.data = {
				"valid": True,
				"id": "talon1718",
				"sim_time_s": round(t_sim, 3),
				"utc": datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
				"lat": round(lat, 8),
				"lon": round(lon, 8),
				"pos_east_m": round(float(p[0]), 3),
				"pos_north_m": round(float(p[1]), 3),
				"alt_amsl_m": round(self.alt0 + p[2], 2),
				"alt_rel_m": round(p[2], 2),
				"heading_deg": round(heading, 2),
				"ground_speed_mps": round(float(np.hypot(v[0], v[1])), 2),
				"vn_mps": round(float(v[1]), 2),
				"ve_mps": round(float(v[0]), 2),
				"vd_mps": round(float(-v[2]), 2),
				"roll_deg": round(roll, 2),
				"pitch_deg": round(-pitch, 2),
				"mode": mode,
				"target_speed": round(tgt_speed, 1),
				"target_alt": round(tgt_alt, 1),
			}

	def snapshot(self):
		with self.lock:
			return dict(self.data)


def serve_http(telemetry, profile, port):
	class Handler(BaseHTTPRequestHandler):
		def do_GET(self):
			if self.path.split("?")[0] not in ("/", "/target"):
				self.send_error(404)
				return

			body = json.dumps(telemetry.snapshot()).encode()
			self.send_response(200)
			self.send_header("Content-Type", "application/json")
			self.send_header("Access-Control-Allow-Origin", "*")
			self.send_header("Content-Length", str(len(body)))
			self.end_headers()
			self.wfile.write(body)

		def do_POST(self):
			if self.path.split("?")[0] in ("/cmd", "/control", "/mode"):
				try:
					length = int(self.headers.get("Content-Length", 0))
					data = json.loads(self.rfile.read(length).decode("utf-8")) if length > 0 else {}
					profile.apply_cmd(data)
					resp = json.dumps({"status": "ok", "applied": profile.get_cmd_status()}).encode("utf-8")
					self.send_response(200)
					self.send_header("Content-Type", "application/json")
					self.send_header("Access-Control-Allow-Origin", "*")
					self.send_header("Content-Length", str(len(resp)))
					self.end_headers()
					self.wfile.write(resp)
				except Exception as e:
					err_msg = json.dumps({"status": "error", "message": str(e)}).encode("utf-8")
					self.send_response(400)
					self.send_header("Content-Type", "application/json")
					self.send_header("Content-Length", str(len(err_msg)))
					self.end_headers()
					self.wfile.write(err_msg)
			else:
				self.send_error(404)

		def log_message(self, *args):
			pass

	server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
	threading.Thread(target=server.serve_forever, daemon=True).start()
	return server


def send_adsb(link, d):
	flags = (mav.ADSB_FLAGS_VALID_COORDS | mav.ADSB_FLAGS_VALID_ALTITUDE | mav.ADSB_FLAGS_VALID_HEADING
		 | mav.ADSB_FLAGS_VALID_VELOCITY | mav.ADSB_FLAGS_VALID_CALLSIGN)
	link.mav.adsb_vehicle_send(0x7A1718, int(d["lat"] * 1e7), int(d["lon"] * 1e7),
				   mav.ADSB_ALTITUDE_TYPE_GEOMETRIC, int(d["alt_amsl_m"] * 1000),
				   int(d["heading_deg"] * 100) % 36000, int(d["ground_speed_mps"] * 100),
				   int(-d["vd_mps"] * 100), b"TALON", mav.ADSB_EMITTER_TYPE_UAV, 0, flags, 0)


# -------------------------------------------------------------------- main ---

def main():
	ap = argparse.ArgumentParser()
	ap.add_argument("--world", default="ankara")
	ap.add_argument("--model", default="talon1718_1")
	ap.add_argument("--alt", type=float, default=100.0, help="circuit altitude above the runway [m]")
	ap.add_argument("--speed", type=float, default=25.0, help="[m/s]")
	ap.add_argument("--side", type=float, default=2000.0, help="square side [m]")
	ap.add_argument("--bank", type=float, default=35.0, help="bank angle in the corners [deg]")
	ap.add_argument("--accel", type=float, default=4.0, help="take-off acceleration [m/s^2]")
	ap.add_argument("--climb-angle", type=float, default=8.0, help="[deg]")
	ap.add_argument("--heading", type=float, default=None, help="first side [deg true], default: runway")
	ap.add_argument("--pattern", choices=["weave", "square"], default="weave",
			help="flight path pattern: 'weave' (continuous serpentine S-turns) or 'square' (square circuit)")
	ap.add_argument("--weave-amp", type=float, default=26.0, help="lateral weave amplitude [m] (controls bank angle)")
	ap.add_argument("--weave-wavelength", type=float, default=550.0, help="weave cycle wavelength [m]")
	ap.add_argument("--rate", type=float, default=2.0, help="telemetry rate [Hz]")
	ap.add_argument("--http-port", type=int, default=8000)
	ap.add_argument("--adsb", default="udpout:127.0.0.1:14580", help="MAVLink target for ADSB_VEHICLE, '' = off")
	a = ap.parse_args()

	sdf = open(os.path.join(SIM_DIR, "worlds", a.world + ".sdf")).read()
	lat0 = float(re.search(r"<latitude_deg>([^<]+)", sdf).group(1))
	lon0 = float(re.search(r"<longitude_deg>([^<]+)", sdf).group(1))
	alt0 = float(re.search(r"<elevation>([^<]+)", sdf).group(1))
	env = open(os.path.join(SIM_DIR, "worlds", a.world + ".env")).read()
	pose = [float(v) for v in re.search(r"TARGET_POSE_DEFAULT=([^\n]+)", env).group(1).split(",")]

	if a.heading is None:
		a.heading = 90 - math.degrees(pose[5])

	a.z0 = pose[2]

	profile = Profile(a)
	telemetry = Telemetry(lat0, lon0, alt0)
	serve_http(telemetry, profile, a.http_port)
	adsb = mavutil.mavlink_connection(a.adsb, source_system=250, source_component=1) if a.adsb else None

	node = Node()
	state = {}
	node.subscribe(Clock, f"/world/{a.world}/clock", lambda m: state.__setitem__("t", m.sim.sec + m.sim.nsec * 1e-9))

	def on_pose(msg):
		for p in msg.pose:
			if p.name == a.model:
				state["pose"] = p

	node.subscribe(Pose_V, f"/world/{a.world}/dynamic_pose/info", on_pose)
	cmd_pub = node.advertise(f"/model/{a.model}/cmd_vel", Twist)

	if profile.pattern == "square":
		print(f"target: {a.side:.0f} m square, {a.alt:.0f} m, {a.speed:.0f} m/s, turn radius {profile.radius:.0f} m, "
		      f"lap {profile.lap:.0f} m; HTTP :{a.http_port}/target, ADS-B {a.adsb or 'off'}", flush=True)
	else:
		print(f"target: continuous weave (amp {profile.wave_amp:.1f} m, cycle {profile.wave_len:.0f} m, ~12 deg bank), "
		      f"{a.alt:.0f} m, {a.speed:.0f} m/s; HTTP :{a.http_port}/target, ADS-B {a.adsb or 'off'}", flush=True)

	while "pose" not in state or "t" not in state:
		time.sleep(0.1)

	t_start = state["t"]
	last_tel, last_print = -1e9, -1e9
	prev = None

	while True:
		time.sleep(0.02)
		t_sim = state["t"]
		t = t_sim - t_start
		pp = state["pose"]
		p = np.array([pp.position.x, pp.position.y, pp.position.z])
		R = quat_to_rot(pp.orientation.w, pp.orientation.x, pp.orientation.y, pp.orientation.z)

		p_d, v_d, R_d, w_d = profile.state(t)
		v_cmd_world = v_d + 1.5 * (p_d - p)
		w_cmd_body = 4.0 * rot_log(R.T @ R_d) + R.T @ w_d
		msg = Twist()
		(msg.linear.x, msg.linear.y, msg.linear.z) = R.T @ v_cmd_world
		(msg.angular.x, msg.angular.y, msg.angular.z) = w_cmd_body
		cmd_pub.publish(msg)

		if t_sim - last_tel >= 1.0 / a.rate:
			v = (p - prev[1]) / (t_sim - prev[0]) if prev and t_sim > prev[0] else v_d
			prev = (t_sim, p)
			telemetry.update(t_sim, p, v, R, mode=profile.mode, tgt_speed=profile.target_speed, tgt_alt=profile.target_alt)
			last_tel = t_sim

			if adsb:
				send_adsb(adsb, telemetry.snapshot())

		if t_sim - last_print >= 10:
			d = telemetry.snapshot()
			if d["valid"]:
				print(f"t={t:6.0f}s  alt {d['alt_rel_m']:6.1f} m  speed {d['ground_speed_mps']:5.1f} m/s  "
				      f"heading {d['heading_deg']:5.1f}  pos error {np.linalg.norm(p_d - p):5.2f} m", flush=True)
			last_print = t_sim


if __name__ == "__main__":
	main()
