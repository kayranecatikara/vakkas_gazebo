#!/usr/bin/env python3
"""
Tactical Web Ground Control & Evaluation Station (Web C2) for Octopus Interceptor & Talon Target.
Powered by FastAPI, WebSockets, HTML5 Canvas Radar, and Chart.js.

Usage:
  python3 tools/web_gcs.py [--port 8080]
"""

import os
import sys
import json
import math
import time
import asyncio
import threading
import subprocess
import urllib.request
from collections import deque

os.environ["GZ_IP"] = "127.0.0.1"
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
os.environ["MAVLINK20"] = "1"

import numpy as np
from pymavlink import mavutil
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
PX4_DIR = os.path.abspath(os.path.join(SIM_DIR, "..", "..", ".."))

app = FastAPI(title="Octopus Tactical Web GCS")

# Shared state
state_lock = threading.Lock()
telemetry_data = {
    "interceptor": {
        "connected": False,
        "armed": False,
        "mode": "STANDBY",
        "pos": [0.0, 0.0, 0.0], # North, East, Down
        "vel": [0.0, 0.0, 0.0],
        "att": [0.0, 0.0, 0.0], # Roll, Pitch, Yaw in degrees
        "speed": 0.0,
        "alt": 0.0,
        "last_update": 0.0
    },
    "talon": {
        "connected": False,
        "mode": "square",
        "target_speed": 22.0,
        "target_alt": 100.0,
        "pos": [0.0, 0.0, 0.0], # North, East, Down
        "vel": [0.0, 0.0, 0.0],
        "att": [0.0, 0.0, 0.0],
        "speed": 0.0,
        "alt": 0.0,
        "heading": 0.0,
        "last_update": 0.0
    },
    "tactical": {
        "separation_distance": 0.0,
        "closing_speed": 0.0,
        "guidance_phase": "STANDBY",
        "fps_mode": 50,
        "lock_percent": 0.0,
        "net_deployed": False
    }
}

lock_start_time = None
history_times = deque(maxlen=200)
history_dist = deque(maxlen=200)
history_v_int = deque(maxlen=200)
history_v_tgt = deque(maxlen=200)
int_trail = deque(maxlen=1000)
tgt_trail = deque(maxlen=1000)

# ---------------------------------------------------- Gazebo Transport Worker ---

def gz_transport_worker_thread(world="ankara", interceptor_name="interceptor_0", target_name="talon1718_1"):
    global telemetry_data
    try:
        from gz.transport13 import Node
        from gz.msgs10.pose_v_pb2 import Pose_V
    except Exception as e:
        print("Gazebo transport import notice in Web GCS:", e, flush=True)
        return

    node = Node()
    prev_pos = None
    prev_t = 0.0

    def on_pose(msg):
        nonlocal prev_pos, prev_t
        now = time.time()
        for p in msg.pose:
            if p.name == interceptor_name:
                pos = p.position
                ori = p.orientation

                # Quaternion -> Roll, Pitch, Yaw
                qx, qy, qz, qw = ori.x, ori.y, ori.z, ori.w
                sinr_cosp = 2 * (qw * qx + qy * qz)
                cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
                roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

                sinp = 2 * (qw * qy - qz * qx)
                pitch = math.degrees(math.asin(max(-1.0, min(1.0, sinp))))

                siny_cosp = 2 * (qw * qz + qx * qy)
                cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
                yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))

                # Numerical velocity from pose
                vx, vy, vz = 0.0, 0.0, 0.0
                if prev_pos is not None and now > prev_t:
                    dt = max(0.001, now - prev_t)
                    vx = (pos.x - prev_pos[0]) / dt
                    vy = (pos.y - prev_pos[1]) / dt
                    vz = (pos.z - prev_pos[2]) / dt

                prev_pos = (pos.x, pos.y, pos.z)
                prev_t = now
                spd = float(math.hypot(vx, vy))

                with state_lock:
                    inter = telemetry_data["interceptor"]
                    inter["connected"] = True
                    inter["last_update"] = now
                    # Gazebo ENU -> NED for local convention:
                    inter["pos"] = [round(pos.y, 3), round(pos.x, 3), round(-pos.z, 3)]
                    inter["vel"] = [round(vy, 3), round(vx, 3), round(-vz, 3)]
                    inter["speed"] = round(spd, 2)
                    inter["alt"] = round(pos.z, 2)
                    inter["att"] = [round(roll, 1), round(pitch, 1), round(yaw, 1)]
                    int_trail.append([round(pos.x, 2), round(pos.y, 2), round(pos.z, 2)])

    topic = f"/world/{world}/dynamic_pose/info"
    node.subscribe(Pose_V, topic, on_pose)
    while True:
        time.sleep(1.0)


# ----------------------------------------------------------- MAVLink Worker ---

def mavlink_worker_thread(primary_port=14545, fallback_ports=(14540, 14550, 14551)):
    global telemetry_data
    ports_to_try = [primary_port] + list(fallback_ports)
    link = None
    port_idx = 0

    while True:
        if link is None:
            port = ports_to_try[port_idx % len(ports_to_try)]
            try:
                link = mavutil.mavlink_connection(f"udpin:0.0.0.0:{port}")
            except Exception:
                port_idx += 1
                time.sleep(1.0)
                continue

        try:
            msg = link.recv_match(blocking=True, timeout=1.0)
            if not msg:
                continue

            mtype = msg.get_type()
            now = time.time()

            with state_lock:
                inter = telemetry_data["interceptor"]
                inter["connected"] = True
                inter["last_update"] = now

                if mtype == "LOCAL_POSITION_NED":
                    inter["pos"] = [round(msg.x, 3), round(msg.y, 3), round(msg.z, 3)]
                    inter["vel"] = [round(msg.vx, 3), round(msg.vy, 3), round(msg.vz, 3)]
                    inter["speed"] = round(float(np.hypot(msg.vx, msg.vy)), 2)
                    inter["alt"] = round(-msg.z, 2)
                    int_trail.append([round(msg.y, 2), round(msg.x, 2), round(-msg.z, 2)])

                elif mtype == "ATTITUDE":
                    inter["att"] = [
                        round(math.degrees(msg.roll), 1),
                        round(math.degrees(msg.pitch), 1),
                        round(math.degrees(msg.yaw), 1)
                    ]

                elif mtype == "HEARTBEAT":
                    inter["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    custom = msg.custom_mode
                    main_mode = (custom >> 16) & 0xFF
                    sub_mode = (custom >> 24) & 0xFF
                    if main_mode == 1:
                        inter["mode"] = "MANUAL"
                    elif main_mode == 2:
                        inter["mode"] = "ALTCTL"
                    elif main_mode == 3:
                        inter["mode"] = "POSCTL"
                    elif main_mode == 4:
                        if sub_mode == 3:
                            inter["mode"] = "HOLD"
                        elif sub_mode == 4:
                            inter["mode"] = "TAKEOFF"
                        elif sub_mode == 5:
                            inter["mode"] = "LAND"
                        else:
                            inter["mode"] = "AUTO"
                    elif main_mode == 6:
                        inter["mode"] = "INTERCEPT"
                    else:
                        inter["mode"] = f"MODE_{main_mode}"

        except Exception:
            link = None
            port_idx += 1
            time.sleep(0.5)


# -------------------------------------------------------- Target Poller ---

def target_poller_thread(http_port=8000, rate=20.0):
    global telemetry_data
    dt = 1.0 / rate
    url = f"http://127.0.0.1:{http_port}/target"

    while True:
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "WebGCS"})
            with urllib.request.urlopen(req, timeout=0.4) as resp:
                if resp.status == 200:
                    d = json.loads(resp.read().decode("utf-8"))
                    now = time.time()
                    with state_lock:
                        talon = telemetry_data["talon"]
                        talon["connected"] = d.get("valid", False)
                        if talon["connected"]:
                            talon["last_update"] = now
                            # Position
                            if "pos_east_m" in d and "pos_north_m" in d:
                                de = d["pos_east_m"]
                                dn = d["pos_north_m"]
                            else:
                                lat0, lon0 = 39.930898, 32.729591
                                dn = (d.get("lat", lat0) - lat0) * 111132.95
                                de = (d.get("lon", lon0) - lon0) * (111319.49 * math.cos(math.radians(lat0)))

                            alt = d.get("alt_rel_m", 0.0)
                            talon["pos"] = [round(dn, 3), round(de, 3), round(-alt, 3)]
                            talon["vel"] = [
                                round(d.get("vn_mps", 0.0), 3),
                                round(d.get("ve_mps", 0.0), 3),
                                round(d.get("vd_mps", 0.0), 3)
                            ]
                            talon["speed"] = round(d.get("ground_speed_mps", 0.0), 2)
                            talon["alt"] = round(alt, 2)
                            talon["heading"] = round(d.get("heading_deg", 0.0), 1)
                            talon["mode"] = d.get("mode", "square")
                            talon["target_speed"] = round(d.get("target_speed", 22.0), 1)
                            talon["target_alt"] = round(d.get("target_alt", 100.0), 1)
                            talon["att"] = [
                                round(d.get("roll_deg", 0.0), 1),
                                round(d.get("pitch_deg", 0.0), 1),
                                round(d.get("heading_deg", 0.0), 1)
                            ]
                            tgt_trail.append([round(de, 2), round(dn, 2), round(alt, 2)])
        except Exception:
            with state_lock:
                telemetry_data["talon"]["connected"] = False

        time.sleep(dt)


# ---------------------------------------------------- Tactical Computations ---

def tactical_loop_thread(rate=30.0):
    global telemetry_data, lock_start_time
    dt = 1.0 / rate
    t0 = time.time()

    while True:
        now = time.time()
        with state_lock:
            inter = telemetry_data["interceptor"]
            talon = telemetry_data["talon"]
            tac = telemetry_data["tactical"]

            # Connection timeouts
            if now - inter["last_update"] > 2.0:
                inter["connected"] = False
            if now - talon["last_update"] > 2.0:
                talon["connected"] = False

            if inter["connected"] and talon["connected"]:
                p_int = np.array(inter["pos"])
                p_tgt = np.array(talon["pos"])
                v_int = np.array(inter["vel"])
                v_tgt = np.array(talon["vel"])

                d_vec = p_tgt - p_int
                dist = float(np.linalg.norm(d_vec))
                r_unit = d_vec / max(dist, 0.01)
                closing_vel = float(np.dot(v_int - v_tgt, r_unit))

                tac["separation_distance"] = round(dist, 2)
                tac["closing_speed"] = round(closing_vel, 2)

                # Phase determination
                if dist > 40.0:
                    tac["guidance_phase"] = "MIDCOURSE (GPS İntikali)"
                    tac["fps_mode"] = 50
                elif dist > 20.0:
                    tac["guidance_phase"] = "TERMINAL (Optik 50 FPS)"
                    tac["fps_mode"] = 50
                elif dist > 5.5:
                    tac["guidance_phase"] = "TERMINAL (Yüksek Hız 80 FPS)"
                    tac["fps_mode"] = 80
                else:
                    tac["guidance_phase"] = "AĞ MENZİLİNDE (KİLİTLİ)"
                    tac["fps_mode"] = 80

                # Net lock progress
                if dist <= 5.5:
                    if lock_start_time is None:
                        lock_start_time = now
                    elapsed = now - lock_start_time
                    pct = min(100.0, (elapsed / 0.40) * 100.0)
                    tac["lock_percent"] = round(pct, 1)
                    if pct >= 100.0:
                        tac["net_deployed"] = True
                else:
                    lock_start_time = None
                    tac["lock_percent"] = 0.0

                # History
                t_rel = round(now - t0, 2)
                history_times.append(t_rel)
                history_dist.append(round(dist, 2))
                history_v_int.append(inter["speed"])
                history_v_tgt.append(talon["speed"])

        time.sleep(dt)


# ----------------------------------------------------------- REST Endpoints ---

@app.post("/api/talon/cmd")
async def talon_cmd(payload: dict):
    """Forward command to target_sim.py HTTP server."""
    try:
        url = "http://127.0.0.1:8000/cmd"
        req = urllib.request.Request(
            url,
            data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(req, timeout=1.0) as resp:
            return JSONResponse({"status": "ok", "applied": json.loads(resp.read().decode())})
    except Exception as e:
        return JSONResponse({"status": "error", "message": str(e)}, status_code=500)


@app.post("/api/interceptor/cmd")
async def interceptor_cmd(payload: dict):
    """Execute command via px4cmd.sh."""
    action = payload.get("action", "")
    px4cmd = os.path.join(SCRIPT_DIR, "px4cmd.sh")

    if action == "takeoff":
        subprocess.Popen([px4cmd, "0", "commander", "arm"])
        cmd = [px4cmd, "0", "commander", "takeoff"]
    elif action == "intercept":
        cmd = [px4cmd, "0", "commander", "mode", "ext1"]
    elif action == "hold":
        cmd = [px4cmd, "0", "commander", "mode", "posctl"]
    elif action == "land":
        cmd = [px4cmd, "0", "commander", "mode", "auto:land"]
    else:
        return JSONResponse({"status": "error", "message": f"Unknown action: {action}"}, status_code=400)

    try:
        subprocess.Popen(cmd)
        return JSONResponse({"status": "ok", "action": action})
    except Exception as e:
        return JSONResponse({"status": "error", "message": str(e)}, status_code=500)


@app.post("/api/tools/hud")
async def launch_hud():
    """Launch HUD viewer."""
    try:
        cmd = [os.path.join(SCRIPT_DIR, "view.sh")]
        subprocess.Popen(cmd)
        return JSONResponse({"status": "ok", "message": "HUD başlatıldı"})
    except Exception as e:
        return JSONResponse({"status": "error", "message": str(e)}, status_code=500)


@app.post("/api/tools/qgc")
async def launch_qgc():
    """Launch QGroundControl."""
    qgc_path = "/home/kayra/Applications/QGroundControl-x86_64.AppImage"
    if os.path.exists(qgc_path):
        subprocess.Popen([qgc_path])
        return JSONResponse({"status": "ok", "message": "QGroundControl başlatıldı"})
    return JSONResponse({"status": "error", "message": "QGC AppImage bulunamadı"}, status_code=404)


# ------------------------------------------------------------- WebSocket ---

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            with state_lock:
                packet = {
                    "interceptor": telemetry_data["interceptor"],
                    "talon": telemetry_data["talon"],
                    "tactical": telemetry_data["tactical"],
                    "trails": {
                        "interceptor": list(int_trail),
                        "talon": list(tgt_trail)
                    },
                    "charts": {
                        "times": list(history_times),
                        "dist": list(history_dist),
                        "v_int": list(history_v_int),
                        "v_tgt": list(history_v_tgt)
                    }
                }
            await ws.send_text(json.dumps(packet))
            await asyncio.sleep(0.04) # 25 Hz stream
    except WebSocketDisconnect:
        pass
    except Exception:
        pass


# --------------------------------------------------------- Single Page HTML ---

HTML_CONTENT = """<!DOCTYPE html>
<html lang="tr">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>OCTOPUS C2 // Taktik Görev & Önleme İstasyonu</title>
  <script src="https://cdn.tailwindcss.com"></script>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css">
  <style>
    @import url('https://fonts.googleapis.com/css2?family=JetBrains+Mono:wght@400;600;800&family=Rajdhani:wght@500;600;700&display=swap');
    
    body {
      background-color: #0b0f17;
      color: #e2e8f0;
      font-family: 'Rajdhani', sans-serif;
      overflow-x: hidden;
    }
    .mono { font-family: 'JetBrains Mono', monospace; }
    
    /* Neon Glow Effects */
    .glow-cyan { text-shadow: 0 0 10px rgba(0, 229, 255, 0.6); }
    .glow-green { text-shadow: 0 0 10px rgba(0, 255, 136, 0.6); }
    .glow-amber { text-shadow: 0 0 10px rgba(255, 183, 0, 0.6); }
    .glow-red { text-shadow: 0 0 10px rgba(255, 59, 48, 0.6); }
    
    .card-glass {
      background: rgba(16, 23, 38, 0.85);
      backdrop-filter: blur(8px);
      border: 1px solid rgba(45, 55, 72, 0.7);
    }
    
    /* Custom Scrollbar */
    ::-webkit-scrollbar { width: 6px; height: 6px; }
    ::-webkit-scrollbar-track { background: #0b0f17; }
    ::-webkit-scrollbar-thumb { background: #2d3748; border-radius: 3px; }
  </style>
</head>
<body class="p-3">

  <!-- TOP HEADER -->
  <header class="card-glass rounded-lg p-3 mb-3 flex flex-wrap items-center justify-between border-b-2 border-cyan-500/30">
    <div class="flex items-center space-x-3">
      <div class="w-10 h-10 rounded bg-cyan-950 border border-cyan-400 flex items-center justify-center text-cyan-400 font-bold text-xl">
        <i class="fa-solid fa-crosshairs animate-pulse"></i>
      </div>
      <div>
        <h1 class="text-xl font-bold tracking-wider text-cyan-400">OCTOPUS INTERCEPTOR C2</h1>
        <p class="text-xs text-slate-400 tracking-widest font-mono">100 HZ PREDICTIVE GUIDANCE // TACTICAL EVALUATION BENCH</p>
      </div>
    </div>

    <!-- Quick Status Badges -->
    <div class="flex items-center space-x-3 text-xs mono">
      <button onclick="launchHUD()" class="bg-slate-800 hover:bg-slate-700 text-slate-200 px-3 py-1.5 rounded border border-slate-600 transition flex items-center space-x-1.5">
        <i class="fa-solid fa-video text-amber-400"></i><span>HUD VİZÖRÜ</span>
      </button>
      <button onclick="launchQGC()" class="bg-slate-800 hover:bg-slate-700 text-slate-200 px-3 py-1.5 rounded border border-slate-600 transition flex items-center space-x-1.5">
        <i class="fa-solid fa-satellite-dish text-blue-400"></i><span>QGROUNDCONTROL</span>
      </button>

      <div id="badge-talon" class="bg-red-950/70 border border-red-600 text-red-300 px-3 py-1.5 rounded flex items-center space-x-2">
        <span class="w-2 h-2 rounded-full bg-red-500 animate-ping"></span>
        <span id="txt-talon-badge">TALON: BEKLENİYOR</span>
      </div>

      <div id="badge-interceptor" class="bg-red-950/70 border border-red-600 text-red-300 px-3 py-1.5 rounded flex items-center space-x-2">
        <span class="w-2 h-2 rounded-full bg-red-500 animate-ping"></span>
        <span id="txt-int-badge">ÖNLEYİCİ: BEKLENİYOR</span>
      </div>
    </div>
  </header>

  <!-- MAIN WORKSPACE: 3 COLUMNS -->
  <div class="grid grid-cols-12 gap-3 mb-3">

    <!-- LEFT COLUMN: TALON CONTROLLER (3 Cols) -->
    <div class="col-span-12 lg:col-span-3 space-y-3">
      
      <!-- Scenario Selector -->
      <div class="card-glass rounded-lg p-3.5 border-l-4 border-red-500">
        <div class="flex items-center justify-between mb-2.5">
          <h2 class="text-base font-bold text-red-400 tracking-wide uppercase"><i class="fa-solid fa-plane text-sm mr-2"></i>Hedef Talon 1718 Senaryoları</h2>
          <span id="talon-mode-badge" class="mono text-xs bg-red-900/60 text-red-200 px-2 py-0.5 rounded border border-red-700">SQUARE</span>
        </div>
        
        <div class="grid grid-cols-2 gap-2 text-xs font-semibold">
          <button onclick="setTalonMode('square')" class="bg-slate-800 hover:bg-red-900/50 hover:border-red-500 p-2 rounded border border-slate-700 transition text-left flex items-center space-x-2">
            <i class="fa-regular fa-square text-red-400"></i><span>Kare Devriye</span>
          </button>
          <button onclick="setTalonMode('weave')" class="bg-slate-800 hover:bg-red-900/50 hover:border-red-500 p-2 rounded border border-slate-700 transition text-left flex items-center space-x-2">
            <i class="fa-solid fa-water text-amber-400"></i><span>S-Kaçış (Weave)</span>
          </button>
          <button onclick="setTalonMode('circle')" class="bg-slate-800 hover:bg-red-900/50 hover:border-red-500 p-2 rounded border border-slate-700 transition text-left flex items-center space-x-2">
            <i class="fa-solid fa-arrows-spin text-purple-400"></i><span>Dairesel Orbit</span>
          </button>
          <button onclick="setTalonMode('straight')" class="bg-slate-800 hover:bg-red-900/50 hover:border-red-500 p-2 rounded border border-slate-700 transition text-left flex items-center space-x-2">
            <i class="fa-solid fa-arrow-right-long text-cyan-400"></i><span>Düz Hat Kaçış</span>
          </button>
          <button onclick="setTalonMode('dive')" class="bg-red-950/70 hover:bg-red-900 hover:border-red-400 p-2 rounded border border-red-700 transition text-left flex items-center space-x-2">
            <i class="fa-solid fa-arrow-trend-down text-red-400"></i><span>Acil Dalış (-30m)</span>
          </button>
          <button onclick="setTalonMode('climb')" class="bg-emerald-950/70 hover:bg-emerald-900 hover:border-emerald-400 p-2 rounded border border-emerald-700 transition text-left flex items-center space-x-2">
            <i class="fa-solid fa-arrow-trend-up text-emerald-400"></i><span>Tırmanış (+30m)</span>
          </button>
        </div>
      </div>

      <!-- Sliders -->
      <div class="card-glass rounded-lg p-3.5">
        <h3 class="text-sm font-bold text-slate-300 uppercase tracking-wide mb-3"><i class="fa-solid fa-sliders text-xs mr-2"></i>Dinamik Uçuş Parametreleri</h3>
        
        <div class="space-y-3 text-xs mono">
          <div>
            <div class="flex justify-between text-slate-300 mb-1">
              <span>Hedef Sürati:</span>
              <span id="lbl-spd-val" class="font-bold text-amber-400">22.0 m/s (79 km/h)</span>
            </div>
            <input id="slider-spd" type="range" min="14" max="36" step="1" value="22" oninput="onSpeedChange(this.value)" class="w-full accent-amber-400 bg-slate-800 h-1.5 rounded cursor-pointer">
          </div>

          <div>
            <div class="flex justify-between text-slate-300 mb-1">
              <span>Hedef İrtifası:</span>
              <span id="lbl-alt-val" class="font-bold text-cyan-400">100.0 m</span>
            </div>
            <input id="slider-alt" type="range" min="30" max="180" step="5" value="100" oninput="onAltChange(this.value)" class="w-full accent-cyan-400 bg-slate-800 h-1.5 rounded cursor-pointer">
          </div>
        </div>
      </div>

      <!-- Virtual Joystick Control Deck -->
      <div class="card-glass rounded-lg p-3.5 border-l-4 border-amber-500">
        <div class="flex items-center justify-between mb-2">
          <h3 class="text-sm font-bold text-slate-300 uppercase tracking-wide"><i class="fa-solid fa-gamepad text-xs mr-2 text-amber-400"></i>Talon Sanal Joystick</h3>
          <span id="joy-status-badge" class="mono text-[10px] bg-slate-800 text-slate-400 px-2 py-0.5 rounded border border-slate-700">MERKEZDE</span>
        </div>
        
        <div class="flex flex-col items-center">
          <!-- Joystick Base Container -->
          <div class="relative w-44 h-44 my-1 select-none flex items-center justify-center">
            <!-- Background Ring & Reticle -->
            <div id="joy-base" class="w-40 h-40 rounded-full bg-slate-950/90 border-2 border-slate-700/80 relative overflow-hidden shadow-[inset_0_0_20px_rgba(0,0,0,0.8)] cursor-grab active:cursor-grabbing select-none" style="touch-action: none;">
              <!-- Radial Grid Lines -->
              <div class="absolute inset-0 flex items-center justify-center pointer-events-none">
                <div class="w-full h-[1px] bg-slate-800"></div>
                <div class="h-full w-[1px] bg-slate-800 absolute"></div>
                <div class="w-24 h-24 rounded-full border border-slate-800/80 absolute"></div>
                <div class="w-12 h-12 rounded-full border border-dashed border-slate-700/60 absolute"></div>
                <!-- Direction Labels -->
                <span class="absolute top-1 text-[9px] mono text-slate-500 font-bold">TIRMAN</span>
                <span class="absolute bottom-1 text-[9px] mono text-slate-500 font-bold">DAL</span>
                <span class="absolute left-1.5 text-[9px] mono text-slate-500 font-bold">SOL</span>
                <span class="absolute right-1.5 text-[9px] mono text-slate-500 font-bold">SAĞ</span>
              </div>
              
              <!-- Draggable Knob -->
              <div id="joy-knob" class="absolute w-12 h-12 rounded-full bg-gradient-to-br from-red-500 to-amber-600 border-2 border-white/80 shadow-[0_0_15px_rgba(239,68,68,0.7)] flex items-center justify-center pointer-events-none transform -translate-x-1/2 -translate-y-1/2" style="left: 80px; top: 80px;">
                <div class="w-4 h-4 rounded-full bg-white/90 shadow-inner"></div>
              </div>
            </div>
          </div>

          <!-- Axis Values -->
          <div class="w-full grid grid-cols-2 gap-2 text-xs mono mt-1">
            <div class="bg-slate-900/80 p-1.5 rounded border border-slate-800 text-center">
              <span class="text-[10px] text-slate-400 block">DÖNÜŞ ORANI</span>
              <span id="joy-turn-val" class="font-bold text-amber-400">0.00 rad/s</span>
            </div>
            <div class="bg-slate-900/80 p-1.5 rounded border border-slate-800 text-center">
              <span class="text-[10px] text-slate-400 block">TIRMANIŞ</span>
              <span id="joy-climb-val" class="font-bold text-cyan-400">0.0 m/s</span>
            </div>
          </div>

          <button onclick="resetTalonJoystick()" class="w-full mt-2.5 bg-slate-800 hover:bg-slate-700 border border-slate-600 text-slate-200 text-xs py-1.5 rounded font-semibold transition active:scale-95 flex items-center justify-center space-x-1.5">
            <i class="fa-solid fa-arrows-to-dot text-amber-400"></i><span>DÜZELT & SEVİYELE</span>
          </button>
        </div>
      </div>

    </div>

    <!-- CENTER COLUMN: INTERACTIVE 3D/ORTHOGONAL RADAR (6 Cols) -->
    <div class="col-span-12 lg:col-span-6 card-glass rounded-lg p-3.5 flex flex-col">
      <div class="flex flex-wrap items-center justify-between gap-2 mb-2">
        <div class="flex items-center space-x-2">
          <span class="w-3 h-3 rounded-full bg-emerald-400 animate-pulse"></span>
          <h2 class="text-base font-bold text-cyan-400 tracking-wider uppercase">3B İNTERAKTİF TAKTİK RADAR</h2>
        </div>
        
        <!-- View Mode & Control Buttons -->
        <div class="flex flex-wrap items-center gap-1.5 text-xs mono">
          <!-- View Modes -->
          <div class="bg-slate-900 p-0.5 rounded border border-slate-700 flex space-x-1">
            <button id="btn-mode-xy" onclick="setRadarMode('xy')" class="px-2 py-0.5 rounded bg-slate-800 hover:bg-cyan-900 text-slate-300 border border-transparent transition">XY (ÜST)</button>
            <button id="btn-mode-xz" onclick="setRadarMode('xz')" class="px-2 py-0.5 rounded bg-slate-800 hover:bg-cyan-900 text-slate-300 border border-transparent transition">XZ (YAN)</button>
            <button id="btn-mode-yz" onclick="setRadarMode('yz')" class="px-2 py-0.5 rounded bg-slate-800 hover:bg-cyan-900 text-slate-300 border border-transparent transition">YZ (ÖN)</button>
            <button id="btn-mode-3d" onclick="setRadarMode('3d')" class="px-2 py-0.5 rounded bg-cyan-600 text-white font-bold border border-cyan-400 shadow transition">3B (SERBEST)</button>
          </div>

          <!-- Zoom & Reset -->
          <button onclick="zoomRadar(1.2)" title="Yakınlaş" class="bg-slate-800 border border-slate-600 px-2 py-0.5 rounded hover:bg-slate-700 text-slate-200 font-bold">+</button>
          <button onclick="zoomRadar(0.8)" title="Uzaklaş" class="bg-slate-800 border border-slate-600 px-2 py-0.5 rounded hover:bg-slate-700 text-slate-200 font-bold">-</button>
          <button onclick="focusRadarVehicles()" title="Araçları Ortala" class="bg-slate-800 border border-slate-600 px-2 py-0.5 rounded hover:bg-slate-700 text-cyan-400">ORTALA</button>
          <button onclick="resetRadarView()" title="Görünümü Sıfırla" class="bg-slate-800 border border-slate-600 px-2 py-0.5 rounded hover:bg-slate-700 text-amber-400">SIFIRLA</button>
        </div>
      </div>

      <!-- Radar Canvas Container -->
      <div class="relative flex-1 w-full bg-[#070b12] rounded-lg border border-cyan-900/60 overflow-hidden min-h-[400px] flex items-center justify-center">
        <canvas id="radarCanvas" class="w-full h-full cursor-grab active:cursor-grabbing"></canvas>
        
        <!-- Legend Overlay -->
        <div class="absolute bottom-2 left-2 bg-slate-950/85 p-2 rounded border border-slate-800 text-[11px] mono space-y-1 backdrop-blur-sm pointer-events-none">
          <div class="flex items-center space-x-2">
            <span class="w-2.5 h-2.5 rounded-full bg-[#00ff66] shadow-[0_0_8px_#00ff66] inline-block"></span>
            <span class="text-slate-200 font-semibold">Octopus 100 (Önleyici)</span>
          </div>
          <div class="flex items-center space-x-2">
            <span class="w-2.5 h-2.5 rounded-full bg-[#ff3344] shadow-[0_0_8px_#ff3344] inline-block"></span>
            <span class="text-slate-200 font-semibold">Talon 1718 (Hedef)</span>
          </div>
          <div class="flex items-center space-x-2">
            <span class="w-2.5 h-2.5 border border-dashed border-emerald-400 inline-block"></span>
            <span class="text-slate-400">5 Metre Ağ Yakalama Halkası</span>
          </div>
          <div class="flex items-center space-x-2">
            <span class="w-3 h-0.5 bg-amber-400 inline-block"></span>
            <span class="text-slate-400">Görüş Hattı (LOS)</span>
          </div>
        </div>

        <!-- Mode & Hint Overlay -->
        <div class="absolute top-2 left-2 mono text-[10px] text-slate-400 bg-slate-950/70 px-2 py-1 rounded border border-slate-800/80 pointer-events-none">
          <span id="radar-help-hint">3B Serbest: Sol Tık: Döndür | Sağ Tık/Shift: Kaydır | Tekerlek: Zoom</span>
        </div>

        <div id="radar-fps-badge" class="absolute top-2 right-2 mono text-xs bg-slate-900/80 px-2 py-1 rounded border border-slate-700 text-cyan-400 pointer-events-none">
          60 FPS 3B
        </div>
      </div>
    </div>

    <!-- RIGHT COLUMN: INTERCEPTOR COMMAND & TELEMETRY (3 Cols) -->
    <div class="col-span-12 lg:col-span-3 space-y-3">
      
      <!-- Interceptor Command Deck -->
      <div class="card-glass rounded-lg p-3.5 border-l-4 border-cyan-500">
        <h2 class="text-base font-bold text-cyan-400 tracking-wide uppercase mb-2.5"><i class="fa-solid fa-rocket text-sm mr-2"></i>Önleyici Komuta Masası</h2>
        
        <div class="grid grid-cols-2 gap-2 text-xs font-bold">
          <button onclick="sendInterceptorCmd('takeoff')" class="bg-blue-600 hover:bg-blue-500 text-white p-2.5 rounded transition shadow-lg shadow-blue-900/30 flex items-center justify-center space-x-1.5 active:scale-95">
            <i class="fa-solid fa-plane-departure"></i><span>ARM & KALKIŞ</span>
          </button>
          <button onclick="sendInterceptorCmd('intercept')" class="bg-emerald-600 hover:bg-emerald-500 text-white p-2.5 rounded transition shadow-lg shadow-emerald-900/30 flex items-center justify-center space-x-1.5 active:scale-95">
            <i class="fa-solid fa-bullseye text-amber-300 animate-spin"></i><span>ÖNLEMEYİ BAŞLAT</span>
          </button>
          <button onclick="sendInterceptorCmd('hold')" class="bg-amber-600 hover:bg-amber-500 text-white p-2 rounded transition flex items-center justify-center space-x-1.5 active:scale-95">
            <i class="fa-solid fa-pause"></i><span>HAVADA TUT (HOLD)</span>
          </button>
          <button onclick="sendInterceptorCmd('land')" class="bg-red-700 hover:bg-red-600 text-white p-2 rounded transition flex items-center justify-center space-x-1.5 active:scale-95">
            <i class="fa-solid fa-plane-arrival"></i><span>İNİŞ YAP (LAND)</span>
          </button>
        </div>
      </div>

      <!-- Separation & Fire Control Telemetry Card -->
      <div class="card-glass rounded-lg p-3.5 border border-cyan-500/30">
        <div class="flex items-center justify-between mb-2">
          <span class="text-xs uppercase tracking-wider text-slate-400">Hedefe Olan Mesafe</span>
          <span id="txt-fps-tag" class="mono text-[11px] bg-slate-800 text-amber-300 px-2 py-0.5 rounded border border-slate-700">50 FPS MODU</span>
        </div>
        
        <!-- Big Distance Value -->
        <div class="text-center py-2 bg-slate-950/60 rounded border border-slate-800 mb-3">
          <div id="txt-distance" class="text-4xl font-extrabold mono text-cyan-400 glow-cyan">--- m</div>
          <div id="txt-phase" class="text-xs font-bold uppercase tracking-widest text-slate-400 mt-1">Güdüm Safhası: STANDBY</div>
        </div>

        <!-- Telemetry Items -->
        <div class="space-y-2 mono text-xs">
          <div class="flex justify-between border-b border-slate-800 pb-1">
            <span class="text-slate-400">Kapanma Hızı (Vc):</span>
            <span id="txt-vc" class="font-bold text-slate-200">--- m/s</span>
          </div>
          <div class="flex justify-between border-b border-slate-800 pb-1">
            <span class="text-slate-400">Önleyici İtki (Pitch):</span>
            <span id="txt-pitch" class="font-bold text-slate-200">---°</span>
          </div>
          <div class="flex justify-between border-b border-slate-800 pb-1">
            <span class="text-slate-400">Önleyici Hızı:</span>
            <span id="txt-v-int" class="font-bold text-cyan-400">--- m/s</span>
          </div>
          <div class="flex justify-between">
            <span class="text-slate-400">Talon Hızı:</span>
            <span id="txt-v-tgt" class="font-bold text-red-400">--- m/s</span>
          </div>
        </div>

        <!-- Net Fire Control Lock Meter -->
        <div class="mt-3 pt-3 border-t border-slate-800">
          <div class="flex justify-between text-xs mono mb-1">
            <span class="text-slate-400 font-bold">AĞ FIRLATMA KİLİDİ:</span>
            <span id="txt-lock-pct" class="font-bold text-emerald-400">0%</span>
          </div>
          <div class="w-full bg-slate-900 h-3 rounded-full overflow-hidden border border-slate-700">
            <div id="bar-lock" class="bg-gradient-to-r from-amber-500 to-emerald-400 h-full w-0 transition-all duration-100"></div>
          </div>
          <div id="txt-net-status" class="text-center text-xs font-bold uppercase tracking-wider text-slate-500 mt-1.5">
            AĞ DURUMU: HAZIR
          </div>
        </div>

      </div>

    </div>

  </div>

  <!-- BOTTOM CHARTS: 2 REAL-TIME PERFORMANCE PLOTS -->
  <div class="grid grid-cols-12 gap-3">
    
    <!-- Distance Plot -->
    <div class="col-span-12 lg:col-span-6 card-glass rounded-lg p-3">
      <div class="flex justify-between items-center mb-1">
        <h3 class="text-xs font-bold uppercase tracking-wider text-cyan-400"><i class="fa-solid fa-chart-line mr-1.5"></i>Ayrılma Mesafesi Zaman Eğrisi [m]</h3>
        <span class="text-[10px] mono text-slate-400">Zamanla Kapanma Performansı</span>
      </div>
      <div class="h-44">
        <canvas id="chartDistance"></canvas>
      </div>
    </div>

    <!-- Speed Comparison Plot -->
    <div class="col-span-12 lg:col-span-6 card-glass rounded-lg p-3">
      <div class="flex justify-between items-center mb-1">
        <h3 class="text-xs font-bold uppercase tracking-wider text-amber-400"><i class="fa-solid fa-gauge-high mr-1.5"></i>Hız Kıyaslama Eğrisi [m/s]</h3>
        <span class="text-[10px] mono text-slate-400">Mavi: Octopus // Kırmızı: Talon</span>
      </div>
      <div class="h-44">
        <canvas id="chartSpeed"></canvas>
      </div>
    </div>

  </div>

  <!-- JAVASCRIPT ENGINE -->
  <script>
    // --- WebSocket Telemetry Link ---
    let ws = null;
    let latestData = null;

    function connectWebSocket() {
      const loc = window.location;
      const wsUri = (loc.protocol === "https:" ? "wss:" : "ws:") + "//" + loc.host + "/ws";
      ws = new WebSocket(wsUri);

      ws.onopen = () => {
        console.log("WebSocket connected.");
      };

      ws.onmessage = (evt) => {
        try {
          const data = JSON.parse(evt.data);
          latestData = data;
          updateDashboard(data);
        } catch (e) {
          console.error("Parse error:", e);
        }
      };

      ws.onclose = () => {
        setTimeout(connectWebSocket, 1000);
      };
    }

    // --- Dashboard Updates ---
    function updateDashboard(d) {
      const inter = d.interceptor;
      const talon = d.talon;
      const tac = d.tactical;

      // Status Badges
      const badgeTalon = document.getElementById("badge-talon");
      const txtTalonBadge = document.getElementById("txt-talon-badge");
      if (talon.connected) {
        badgeTalon.className = "bg-emerald-950/70 border border-emerald-600 text-emerald-300 px-3 py-1.5 rounded flex items-center space-x-2";
        txtTalonBadge.innerText = `TALON: AKTİF [${talon.mode.toUpperCase()}]`;
      } else {
        badgeTalon.className = "bg-red-950/70 border border-red-600 text-red-300 px-3 py-1.5 rounded flex items-center space-x-2";
        txtTalonBadge.innerText = "TALON: BAĞLANTI YOK";
      }

      const badgeInt = document.getElementById("badge-interceptor");
      const txtIntBadge = document.getElementById("txt-int-badge");
      if (inter.connected) {
        badgeInt.className = "bg-cyan-950/70 border border-cyan-500 text-cyan-300 px-3 py-1.5 rounded flex items-center space-x-2";
        txtIntBadge.innerText = `ÖNLEYİCİ: ${inter.mode} [ARMED: ${inter.armed ? 'EVET' : 'HAYIR'}]`;
      } else {
        badgeInt.className = "bg-red-950/70 border border-red-600 text-red-300 px-3 py-1.5 rounded flex items-center space-x-2";
        txtIntBadge.innerText = "ÖNLEYİCİ: BAĞLANTI YOK";
      }

      document.getElementById("talon-mode-badge").innerText = talon.mode.toUpperCase();

      // Sliders label
      document.getElementById("lbl-spd-val").innerText = `${talon.speed.toFixed(1)} m/s (${(talon.speed * 3.6).toFixed(0)} km/h)`;
      document.getElementById("lbl-alt-val").innerText = `${talon.alt.toFixed(1)} m`;

      // Always update vehicle-specific metrics independently:
      if (talon.connected) {
        document.getElementById("txt-v-tgt").innerText = `${talon.speed.toFixed(1)} m/s (${(talon.speed * 3.6).toFixed(0)} km/h)`;
      } else {
        document.getElementById("txt-v-tgt").innerText = "--- m/s";
      }

      if (inter.connected) {
        document.getElementById("txt-v-int").innerText = `${inter.speed.toFixed(1)} m/s (${(inter.speed * 3.6).toFixed(0)} km/h)`;
        document.getElementById("txt-pitch").innerText = `${inter.att[1].toFixed(1)}° (Roll: ${inter.att[0].toFixed(1)}°)`;
      } else {
        document.getElementById("txt-v-int").innerText = "--- m/s";
        document.getElementById("txt-pitch").innerText = "---°";
      }

      // Tactical Engagement Telemetry
      if (inter.connected && talon.connected) {
        const dist = tac.separation_distance;
        const distEl = document.getElementById("txt-distance");
        distEl.innerText = `${dist.toFixed(1)} m`;

        if (dist > 40.0) {
          distEl.className = "text-4xl font-extrabold mono text-cyan-400 glow-cyan";
        } else if (dist > 20.0) {
          distEl.className = "text-4xl font-extrabold mono text-amber-400 glow-amber";
        } else if (dist > 5.5) {
          distEl.className = "text-4xl font-extrabold mono text-orange-400 glow-amber";
        } else {
          distEl.className = "text-4xl font-extrabold mono text-emerald-400 glow-green animate-pulse";
        }

        document.getElementById("txt-phase").innerText = `Güdüm Safhası: ${tac.guidance_phase}`;
        document.getElementById("txt-fps-tag").innerText = `${tac.fps_mode} FPS MODU`;
        document.getElementById("txt-vc").innerText = `${tac.closing_speed > 0 ? '+' : ''}${tac.closing_speed.toFixed(1)} m/s (${(tac.closing_speed*3.6).toFixed(0)} km/h)`;

        // Net Fire Control Progress
        const lockPct = tac.lock_percent;
        document.getElementById("txt-lock-pct").innerText = `${lockPct.toFixed(0)}%`;
        document.getElementById("bar-lock").style.width = `${lockPct}%`;

        const netStatus = document.getElementById("txt-net-status");
        if (tac.net_deployed || lockPct >= 100) {
          netStatus.innerText = "💥 AĞ FIRLATILDI! HEDEF YAKALANDI!";
          netStatus.className = "text-center text-xs font-bold uppercase tracking-wider text-emerald-400 glow-green animate-bounce mt-1.5";
        } else if (dist <= 5.5) {
          netStatus.innerText = "🎯 KİLİTLENİLİYOR...";
          netStatus.className = "text-center text-xs font-bold uppercase tracking-wider text-amber-400 glow-amber mt-1.5";
        } else {
          netStatus.innerText = "AĞ DURUMU: HAZIR";
          netStatus.className = "text-center text-xs font-bold uppercase tracking-wider text-slate-500 mt-1.5";
        }
      } else {
        const distEl = document.getElementById("txt-distance");
        distEl.innerText = talon.connected ? "ÖNLEYİCİ BEKLENİYOR" : "--- m";
        distEl.className = "text-2xl font-bold mono text-slate-400";
        document.getElementById("txt-phase").innerText = "Güdüm Safhası: BEKLENİYOR";
        document.getElementById("txt-vc").innerText = "--- m/s";
      }

      // Charts update
      if (d.charts && d.charts.times.length > 0) {
        updateCharts(d.charts);
      }
    }

    // --- Command Dispatchers ---
    function setTalonMode(mode) {
      fetch("/api/talon/cmd", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({mode: mode})
      });
    }

    function onSpeedChange(val) {
      fetch("/api/talon/cmd", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({speed: parseFloat(val)})
      });
    }

    function onAltChange(val) {
      fetch("/api/talon/cmd", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({alt: parseFloat(val)})
      });
    }

    // --- Talon Virtual Joystick Engine ---
    let joyActive = false;
    let joyTurnRate = 0.0;
    let joyClimbRate = 0.0;
    let joySendTimer = null;
    const joyBase = document.getElementById("joy-base");
    const joyKnob = document.getElementById("joy-knob");
    const joyStatus = document.getElementById("joy-status-badge");
    const joyTurnVal = document.getElementById("joy-turn-val");
    const joyClimbVal = document.getElementById("joy-climb-val");

    const JOY_RADIUS = 52.0;
    const JOY_CENTER_X = 80.0;
    const JOY_CENTER_Y = 80.0;

    function updateJoystickVisual(x, y) {
      if (joyKnob) {
        joyKnob.style.left = `${x}px`;
        joyKnob.style.top = `${y}px`;
      }
    }

    function sendJoystickCommand() {
      if (!joyActive) return;
      fetch("/api/talon/cmd", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          mode: "manual",
          turn_rate: joyTurnRate,
          climb_rate: joyClimbRate
        })
      }).catch(() => {});
    }

    function onJoystickPointerDown(e) {
      joyActive = true;
      try { joyBase.setPointerCapture(e.pointerId); } catch(err) {}
      joyStatus.innerText = "MANUEL DÜMEN";
      joyStatus.className = "mono text-[10px] bg-amber-900/80 text-amber-300 px-2 py-0.5 rounded border border-amber-600";
      onJoystickPointerMove(e);
      if (!joySendTimer) {
        joySendTimer = setInterval(sendJoystickCommand, 60);
      }
    }

    function onJoystickPointerMove(e) {
      if (!joyActive) return;
      const rect = joyBase.getBoundingClientRect();
      const pointerX = e.clientX - rect.left;
      const pointerY = e.clientY - rect.top;

      let dx = pointerX - JOY_CENTER_X;
      let dy = pointerY - JOY_CENTER_Y;
      const dist = Math.hypot(dx, dy);

      if (dist > JOY_RADIUS) {
        dx = (dx / dist) * JOY_RADIUS;
        dy = (dy / dist) * JOY_RADIUS;
      }

      updateJoystickVisual(JOY_CENTER_X + dx, JOY_CENTER_Y + dy);

      let nx = dx / JOY_RADIUS;
      let ny = -dy / JOY_RADIUS;

      if (Math.hypot(nx, ny) < 0.08) {
        nx = 0.0;
        ny = 0.0;
      }

      // Left turns Talon left (+turn_rate), right turns right (-turn_rate)
      joyTurnRate = parseFloat((-nx * 0.60).toFixed(2));
      // Up climbs (+climb_rate), down dives (-climb_rate)
      joyClimbRate = parseFloat((ny * 6.0).toFixed(1));

      joyTurnVal.innerText = `${joyTurnRate > 0 ? '+' : ''}${joyTurnRate.toFixed(2)} rad/s`;
      joyClimbVal.innerText = `${joyClimbRate > 0 ? '+' : ''}${joyClimbRate.toFixed(1)} m/s`;
    }

    function onJoystickPointerUp(e) {
      if (!joyActive) return;
      joyActive = false;
      try { joyBase.releasePointerCapture(e.pointerId); } catch(err) {}

      if (joySendTimer) {
        clearInterval(joySendTimer);
        joySendTimer = null;
      }

      // Spring return to center
      joyKnob.style.transition = "all 0.15s ease-out";
      updateJoystickVisual(JOY_CENTER_X, JOY_CENTER_Y);
      setTimeout(() => { if (joyKnob) joyKnob.style.transition = "none"; }, 160);

      joyTurnRate = 0.0;
      joyClimbRate = 0.0;
      joyTurnVal.innerText = "0.00 rad/s";
      joyClimbVal.innerText = "0.0 m/s";

      joyStatus.innerText = "MERKEZDE";
      joyStatus.className = "mono text-[10px] bg-slate-800 text-slate-400 px-2 py-0.5 rounded border border-slate-700";

      fetch("/api/talon/cmd", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode: "manual", turn_rate: 0.0, climb_rate: 0.0 })
      }).catch(() => {});
    }

    if (joyBase) {
      joyBase.addEventListener("pointerdown", onJoystickPointerDown);
      joyBase.addEventListener("pointermove", onJoystickPointerMove);
      joyBase.addEventListener("pointerup", onJoystickPointerUp);
      joyBase.addEventListener("pointercancel", onJoystickPointerUp);
    }

    function resetTalonJoystick() {
      joyActive = false;
      if (joySendTimer) {
        clearInterval(joySendTimer);
        joySendTimer = null;
      }
      updateJoystickVisual(JOY_CENTER_X, JOY_CENTER_Y);
      joyTurnRate = 0.0;
      joyClimbRate = 0.0;
      joyTurnVal.innerText = "0.00 rad/s";
      joyClimbVal.innerText = "0.0 m/s";
      joyStatus.innerText = "DÜZ UÇUŞ";
      joyStatus.className = "mono text-[10px] bg-emerald-900/80 text-emerald-300 px-2 py-0.5 rounded border border-emerald-600";
      fetch("/api/talon/cmd", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ mode: "straight", turn_rate: 0.0, climb_rate: 0.0 })
      });
    }

    function sendManualTurn(rate) {
      fetch("/api/talon/cmd", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({mode: "manual", turn_rate: rate})
      });
    }

    function sendManualClimb(rate) {
      fetch("/api/talon/cmd", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({mode: "manual", climb_rate: rate})
      });
    }

    function sendInterceptorCmd(action) {
      fetch("/api/interceptor/cmd", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({action: action})
      });
    }

    function launchHUD() {
      fetch("/api/tools/hud", {method: "POST"});
    }

    function launchQGC() {
      fetch("/api/tools/qgc", {method: "POST"});
    }

    // --- 3D & Orthogonal Tactical Radar Canvas ---
    const canvas = document.getElementById("radarCanvas");
    const ctx = canvas.getContext("2d");

    let radarMode = "3d"; // '3d', 'xy', 'xz', 'yz'
    let radarZoom = 1.0;
    let panX = 0.0;
    let panY = 0.0;
    let rotX = 0.58;  // Pitch angle (~33 deg)
    let rotY = -0.65; // Yaw angle (~-37 deg)
    let isDragging = false;
    let dragMode = "orbit"; // "orbit" or "pan"
    let lastMouseX = 0;
    let lastMouseY = 0;

    function resizeCanvas() {
      if (canvas && canvas.parentElement) {
        canvas.width = canvas.parentElement.clientWidth;
        canvas.height = canvas.parentElement.clientHeight;
      }
    }
    window.addEventListener("resize", resizeCanvas);
    resizeCanvas();

    function setRadarMode(mode) {
      radarMode = mode;
      ["xy", "xz", "yz", "3d"].forEach(m => {
        const btn = document.getElementById(`btn-mode-${m}`);
        if (!btn) return;
        if (m === mode) {
          btn.className = "px-2 py-0.5 rounded bg-cyan-600 text-white font-bold border border-cyan-400 shadow transition";
        } else {
          btn.className = "px-2 py-0.5 rounded bg-slate-800 hover:bg-cyan-900 text-slate-300 border border-transparent transition";
        }
      });

      const hintEl = document.getElementById("radar-help-hint");
      if (hintEl) {
        if (mode === "3d") {
          hintEl.innerText = "3B Serbest: Sol Tık: Döndür | Sağ Tık/Shift: Kaydır | Tekerlek: Zoom";
        } else if (mode === "xy") {
          hintEl.innerText = "XY (Kuşbakışı): Sol Tık: Kaydır (Doğu-Kuzey) | Tekerlek: Zoom";
        } else if (mode === "xz") {
          hintEl.innerText = "XZ (Yan Profil): Sol Tık: Kaydır (Doğu-İrtifa) | Tekerlek: Zoom";
        } else if (mode === "yz") {
          hintEl.innerText = "YZ (Ön Profil): Sol Tık: Kaydır (Kuzey-İrtifa) | Tekerlek: Zoom";
        }
      }
    }

    function zoomRadar(factor) {
      radarZoom *= factor;
      radarZoom = Math.max(0.15, Math.min(10.0, radarZoom));
    }

    function resetRadarView() {
      radarZoom = 1.0;
      panX = 0.0;
      panY = 0.0;
      rotX = 0.58;
      rotY = -0.65;
    }

    function focusRadarVehicles() {
      panX = 0.0;
      panY = 0.0;
    }

    // Mouse & Context Listeners
    canvas.addEventListener("contextmenu", (e) => e.preventDefault());

    canvas.addEventListener("mousedown", (e) => {
      isDragging = true;
      lastMouseX = e.clientX;
      lastMouseY = e.clientY;
      if (radarMode === "3d" && (e.button === 2 || e.shiftKey)) {
        dragMode = "pan";
      } else if (radarMode === "3d" && e.button === 0) {
        dragMode = "orbit";
      } else {
        dragMode = "pan";
      }
    });

    window.addEventListener("mousemove", (e) => {
      if (!isDragging) return;
      const dx = e.clientX - lastMouseX;
      const dy = e.clientY - lastMouseY;
      lastMouseX = e.clientX;
      lastMouseY = e.clientY;

      if (dragMode === "orbit") {
        rotY += dx * 0.007;
        rotX = Math.max(0.06, Math.min(1.50, rotX + dy * 0.007));
      } else {
        panX += dx;
        panY += dy;
      }
    });

    window.addEventListener("mouseup", () => {
      isDragging = false;
    });

    canvas.addEventListener("wheel", (e) => {
      e.preventDefault();
      const factor = e.deltaY < 0 ? 1.15 : 0.87;
      zoomRadar(factor);
    }, { passive: false });

    // Touch Support
    let touchStartDist = 0;
    canvas.addEventListener("touchstart", (e) => {
      if (e.touches.length === 1) {
        isDragging = true;
        dragMode = (radarMode === "3d") ? "orbit" : "pan";
        lastMouseX = e.touches[0].clientX;
        lastMouseY = e.touches[0].clientY;
      } else if (e.touches.length === 2) {
        isDragging = false;
        touchStartDist = Math.hypot(
          e.touches[0].clientX - e.touches[1].clientX,
          e.touches[0].clientY - e.touches[1].clientY
        );
      }
    });

    canvas.addEventListener("touchmove", (e) => {
      if (e.touches.length === 1 && isDragging) {
        const dx = e.touches[0].clientX - lastMouseX;
        const dy = e.touches[0].clientY - lastMouseY;
        lastMouseX = e.touches[0].clientX;
        lastMouseY = e.touches[0].clientY;
        if (dragMode === "orbit") {
          rotY += dx * 0.007;
          rotX = Math.max(0.06, Math.min(1.50, rotX + dy * 0.007));
        } else {
          panX += dx;
          panY += dy;
        }
      } else if (e.touches.length === 2) {
        const dist = Math.hypot(
          e.touches[0].clientX - e.touches[1].clientX,
          e.touches[0].clientY - e.touches[1].clientY
        );
        if (touchStartDist > 0) {
          zoomRadar(dist / touchStartDist);
          touchStartDist = dist;
        }
      }
    });

    canvas.addEventListener("touchend", () => {
      isDragging = false;
      touchStartDist = 0;
    });

    // 3D / Orthogonal Projection Function
    function project3D(east, north, alt, w, h) {
      const cx = w / 2 + panX;
      const cy = h / 2 + panY;
      const scale = (Math.min(w, h) / 1200.0) * radarZoom;

      if (radarMode === "xy") {
        return {
          x: cx + east * scale,
          y: cy - north * scale,
          scale: scale
        };
      } else if (radarMode === "xz") {
        return {
          x: cx + east * scale,
          y: cy - alt * scale,
          scale: scale
        };
      } else if (radarMode === "yz") {
        return {
          x: cx + north * scale,
          y: cy - alt * scale,
          scale: scale
        };
      } else {
        // 3D Orbital Projection
        const cosY = Math.cos(rotY), sinY = Math.sin(rotY);
        const x1 = east * cosY - north * sinY;
        const y1 = east * sinY + north * cosY;
        const z1 = alt;

        const cosX = Math.cos(rotX), sinX = Math.sin(rotX);
        const x2 = x1;
        const y2 = y1 * cosX - z1 * sinX;
        const z2 = y1 * sinX + z1 * cosX;

        return {
          x: cx + x2 * scale,
          y: cy - z2 * scale,
          scale: scale
        };
      }
    }

    // Main Tactical Radar Render Loop
    function drawRadar() {
      const w = canvas.width;
      const h = canvas.height;
      ctx.clearRect(0, 0, w, h);

      const cx = w / 2 + panX;
      const cy = h / 2 + panY;
      const scale = (Math.min(w, h) / 1200.0) * radarZoom;

      // 1. Draw Grid & Distance Rings
      const ringSteps = [100, 250, 500, 750, 1000];

      if (radarMode === "3d") {
        // 3D Ground Plane Circles at Alt = 0
        ctx.strokeStyle = "rgba(0, 229, 255, 0.12)";
        ctx.lineWidth = 1;
        for (const r of ringSteps) {
          ctx.beginPath();
          for (let a = 0; a <= 2 * Math.PI + 0.05; a += Math.PI / 24) {
            const p = project3D(r * Math.sin(a), r * Math.cos(a), 0, w, h);
            if (a === 0) ctx.moveTo(p.x, p.y);
            else ctx.lineTo(p.x, p.y);
          }
          ctx.stroke();
          // Distance ring label
          const lp = project3D(r, 0, 0, w, h);
          ctx.fillStyle = "rgba(0, 229, 255, 0.4)";
          ctx.font = "10px monospace";
          ctx.fillText(`${r}m`, lp.x + 3, lp.y - 3);
        }

        // 3D Ground Crosshairs
        ctx.strokeStyle = "rgba(0, 229, 255, 0.18)";
        ctx.beginPath();
        const pN = project3D(0, 1100, 0, w, h);
        const pS = project3D(0, -1100, 0, w, h);
        const pE = project3D(1100, 0, 0, w, h);
        const pW = project3D(-1100, 0, 0, w, h);
        ctx.moveTo(pS.x, pS.y); ctx.lineTo(pN.x, pN.y);
        ctx.moveTo(pW.x, pW.y); ctx.lineTo(pE.x, pE.y);
        ctx.stroke();

        // 3D Cardinal Labels
        ctx.fillStyle = "rgba(0, 229, 255, 0.75)";
        ctx.font = "bold 11px Rajdhani, sans-serif";
        ctx.fillText("N (KUZEY)", pN.x - 18, pN.y - 6);
        ctx.fillText("S (GÜNEY)", pS.x - 18, pS.y + 14);
        ctx.fillText("E (DOĞU)", pE.x + 6, pE.y + 4);
        ctx.fillText("W (BATI)", pW.x - 44, pW.y + 4);

      } else if (radarMode === "xy") {
        // 2D Bird's eye Top-Down
        ctx.strokeStyle = "rgba(0, 229, 255, 0.12)";
        ctx.lineWidth = 1;
        for (const r of ringSteps) {
          ctx.beginPath();
          ctx.arc(cx, cy, r * scale, 0, 2 * Math.PI);
          ctx.stroke();
          ctx.fillStyle = "rgba(0, 229, 255, 0.4)";
          ctx.font = "10px monospace";
          ctx.fillText(`${r}m`, cx + r * scale + 4, cy - 4);
        }
        ctx.beginPath();
        ctx.moveTo(cx, 0); ctx.lineTo(cx, h);
        ctx.moveTo(0, cy); ctx.lineTo(w, cy);
        ctx.stroke();

        ctx.fillStyle = "rgba(0, 229, 255, 0.8)";
        ctx.font = "bold 12px Rajdhani, sans-serif";
        ctx.fillText("N (KUZEY)", cx - 24, 20);
        ctx.fillText("S (GÜNEY)", cx - 24, h - 10);
        ctx.fillText("E (DOĞU)", w - 60, cy - 8);
        ctx.fillText("W (BATI)", 10, cy - 8);

      } else {
        // XZ (Side) or YZ (Front) Profile
        ctx.strokeStyle = "rgba(0, 229, 255, 0.3)";
        ctx.lineWidth = 1.5;
        // Ground horizon line (Alt = 0)
        ctx.beginPath();
        ctx.moveTo(0, cy); ctx.lineTo(w, cy);
        ctx.stroke();
        ctx.fillStyle = "rgba(0, 229, 255, 0.6)";
        ctx.font = "10px monospace";
        ctx.fillText("YER SEVİYESİ (0m)", 10, cy + 14);

        // Altitude steps (50m, 100m, 150m, 200m)
        [50, 100, 150, 200].forEach(alt => {
          const sy = cy - alt * scale;
          ctx.strokeStyle = "rgba(0, 229, 255, 0.1)";
          ctx.lineWidth = 1;
          ctx.beginPath();
          ctx.moveTo(0, sy); ctx.lineTo(w, sy);
          ctx.stroke();
          ctx.fillStyle = "rgba(0, 229, 255, 0.45)";
          ctx.fillText(`+${alt}m`, 10, sy - 3);
        });

        // Vertical centerline
        ctx.strokeStyle = "rgba(0, 229, 255, 0.15)";
        ctx.beginPath();
        ctx.moveTo(cx, 0); ctx.lineTo(cx, h);
        ctx.stroke();

        ctx.fillStyle = "rgba(0, 229, 255, 0.8)";
        ctx.font = "bold 12px Rajdhani, sans-serif";
        if (radarMode === "xz") {
          ctx.fillText("E (DOĞU +)", w - 70, cy - 8);
          ctx.fillText("W (BATI -)", 10, cy - 8);
          ctx.fillText("İRTİFA (ALT)", cx + 6, 20);
        } else {
          ctx.fillText("N (KUZEY +)", w - 80, cy - 8);
          ctx.fillText("S (GÜNEY -)", 10, cy - 8);
          ctx.fillText("İRTİFA (ALT)", cx + 6, 20);
        }
      }

      if (!latestData) {
        requestAnimationFrame(drawRadar);
        return;
      }

      const trails = latestData.trails || {};
      const inter = latestData.interceptor;
      const talon = latestData.talon;
      const tac = latestData.tactical || {};

      // 2. Draw Long Fading Ribbon Trails
      // Talon Ribbon Trail (Neon Red)
      if (trails.talon && trails.talon.length > 1) {
        const N = trails.talon.length;
        ctx.save();
        ctx.shadowColor = "#ff2244";
        ctx.shadowBlur = 8;
        ctx.lineWidth = 1.8;
        const segSize = Math.max(1, Math.floor(N / 12));
        for (let i = 0; i < N - 1; i += segSize) {
          const iNext = Math.min(N - 1, i + segSize);
          const alpha = 0.06 + 0.88 * (iNext / N);
          ctx.strokeStyle = `rgba(255, 51, 68, ${alpha.toFixed(2)})`;
          ctx.beginPath();
          for (let j = i; j <= iNext; j++) {
            const pt = trails.talon[j];
            const sp = project3D(pt[0], pt[1], pt[2] || 0, w, h);
            if (j === i) ctx.moveTo(sp.x, sp.y);
            else ctx.lineTo(sp.x, sp.y);
          }
          ctx.stroke();
        }
        ctx.restore();
      }

      // Interceptor Ribbon Trail (Neon Green)
      if (trails.interceptor && trails.interceptor.length > 1) {
        const N = trails.interceptor.length;
        ctx.save();
        ctx.shadowColor = "#00ff66";
        ctx.shadowBlur = 8;
        ctx.lineWidth = 1.8;
        const segSize = Math.max(1, Math.floor(N / 12));
        for (let i = 0; i < N - 1; i += segSize) {
          const iNext = Math.min(N - 1, i + segSize);
          const alpha = 0.06 + 0.88 * (iNext / N);
          ctx.strokeStyle = `rgba(0, 255, 102, ${alpha.toFixed(2)})`;
          ctx.beginPath();
          for (let j = i; j <= iNext; j++) {
            const pt = trails.interceptor[j];
            const sp = project3D(pt[0], pt[1], pt[2] || 0, w, h);
            if (j === i) ctx.moveTo(sp.x, sp.y);
            else ctx.lineTo(sp.x, sp.y);
          }
          ctx.stroke();
        }
        ctx.restore();
      }

      // 3. Ground Drop Lines & Shadows (3D Mode)
      if (radarMode === "3d") {
        if (talon.connected) {
          const tp = project3D(talon.pos[1], talon.pos[0], talon.alt, w, h);
          const gp = project3D(talon.pos[1], talon.pos[0], 0, w, h);
          ctx.strokeStyle = "rgba(255, 51, 68, 0.35)";
          ctx.lineWidth = 1;
          ctx.setLineDash([2, 3]);
          ctx.beginPath();
          ctx.moveTo(tp.x, tp.y); ctx.lineTo(gp.x, gp.y);
          ctx.stroke();
          ctx.setLineDash([]);
          // Shadow ellipse
          ctx.fillStyle = "rgba(255, 51, 68, 0.25)";
          ctx.beginPath();
          ctx.ellipse(gp.x, gp.y, 5, 2.5, 0, 0, 2 * Math.PI);
          ctx.fill();
        }

        if (inter.connected) {
          const ip = project3D(inter.pos[1], inter.pos[0], inter.alt, w, h);
          const gp = project3D(inter.pos[1], inter.pos[0], 0, w, h);
          ctx.strokeStyle = "rgba(0, 255, 102, 0.35)";
          ctx.lineWidth = 1;
          ctx.setLineDash([2, 3]);
          ctx.beginPath();
          ctx.moveTo(ip.x, ip.y); ctx.lineTo(gp.x, gp.y);
          ctx.stroke();
          ctx.setLineDash([]);
          // Shadow ellipse
          ctx.fillStyle = "rgba(0, 255, 102, 0.25)";
          ctx.beginPath();
          ctx.ellipse(gp.x, gp.y, 5, 2.5, 0, 0, 2 * Math.PI);
          ctx.fill();
        }
      }

      // 4. Vehicles Rendering
      let tx = 0, ty = 0, ix = 0, iy = 0;

      // Target Talon (Glowing Neon Red Dot)
      if (talon.connected) {
        const tp = project3D(talon.pos[1], talon.pos[0], talon.alt, w, h);
        tx = tp.x; ty = tp.y;

        // 5m Capture Circle
        ctx.strokeStyle = "rgba(0, 255, 136, 0.8)";
        ctx.lineWidth = 1.5;
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.arc(tx, ty, Math.max(8, 5.0 * scale), 0, 2 * Math.PI);
        ctx.stroke();
        ctx.setLineDash([]);

        // Glowing Red Halo Aura
        const pulse = 1.0 + 0.2 * Math.sin(Date.now() * 0.007);
        const glowR = 18 * pulse;
        const gradTgt = ctx.createRadialGradient(tx, ty, 2, tx, ty, glowR);
        gradTgt.addColorStop(0, "rgba(255, 51, 68, 0.95)");
        gradTgt.addColorStop(0.4, "rgba(255, 51, 68, 0.45)");
        gradTgt.addColorStop(1, "rgba(255, 51, 68, 0)");
        ctx.fillStyle = gradTgt;
        ctx.beginPath();
        ctx.arc(tx, ty, glowR, 0, 2 * Math.PI);
        ctx.fill();

        // White Glowing Core Dot
        ctx.fillStyle = "#ffffff";
        ctx.beginPath();
        ctx.arc(tx, ty, 4.5, 0, 2 * Math.PI);
        ctx.fill();
        ctx.strokeStyle = "#ff2244";
        ctx.lineWidth = 2;
        ctx.stroke();

        // Heading Indicator Arrow
        const radT = (90 - talon.heading) * Math.PI / 180.0;
        ctx.strokeStyle = "#ff4d4d";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(tx, ty);
        ctx.lineTo(tx + 16 * Math.cos(radT), ty - 16 * Math.sin(radT));
        ctx.stroke();

        // Label
        ctx.fillStyle = "#ff6b6b";
        ctx.font = "bold 11px monospace";
        ctx.fillText(`TALON [${talon.speed.toFixed(0)}m/s | ${talon.alt.toFixed(0)}m]`, tx + 14, ty - 6);
      }

      // Interceptor (Glowing Neon Green Dot)
      if (inter.connected) {
        const ip = project3D(inter.pos[1], inter.pos[0], inter.alt, w, h);
        ix = ip.x; iy = ip.y;

        // Glowing Green Halo Aura
        const pulseInt = 1.0 + 0.2 * Math.sin(Date.now() * 0.007 + 1.2);
        const glowRInt = 18 * pulseInt;
        const gradInt = ctx.createRadialGradient(ix, iy, 2, ix, iy, glowRInt);
        gradInt.addColorStop(0, "rgba(0, 255, 102, 0.95)");
        gradInt.addColorStop(0.4, "rgba(0, 255, 102, 0.45)");
        gradInt.addColorStop(1, "rgba(0, 255, 102, 0)");
        ctx.fillStyle = gradInt;
        ctx.beginPath();
        ctx.arc(ix, iy, glowRInt, 0, 2 * Math.PI);
        ctx.fill();

        // White Glowing Core Dot
        ctx.fillStyle = "#ffffff";
        ctx.beginPath();
        ctx.arc(ix, iy, 4.5, 0, 2 * Math.PI);
        ctx.fill();
        ctx.strokeStyle = "#00ff66";
        ctx.lineWidth = 2;
        ctx.stroke();

        // Heading Indicator Arrow
        const intYaw = (inter.att && inter.att[2] !== undefined) ? inter.att[2] : 0.0;
        const radI = (90 - intYaw) * Math.PI / 180.0;
        ctx.strokeStyle = "#00ffaa";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(ix, iy);
        ctx.lineTo(ix + 16 * Math.cos(radI), iy - 16 * Math.sin(radI));
        ctx.stroke();

        // Label
        ctx.fillStyle = "#63e6be";
        ctx.font = "bold 11px monospace";
        ctx.fillText(`OCTOPUS [${inter.speed.toFixed(0)}m/s | ${inter.alt.toFixed(0)}m]`, ix + 14, iy + 14);

        // Line-Of-Sight (LOS) from Interceptor to Talon
        if (talon.connected) {
          ctx.strokeStyle = "rgba(255, 234, 0, 0.75)";
          ctx.lineWidth = 1.5;
          ctx.setLineDash([3, 4]);
          ctx.beginPath();
          ctx.moveTo(ix, iy);
          ctx.lineTo(tx, ty);
          ctx.stroke();
          ctx.setLineDash([]);

          // Midpoint distance label
          const mx = (ix + tx) / 2;
          const my = (iy + ty) / 2;
          ctx.fillStyle = "rgba(255, 234, 0, 0.95)";
          ctx.font = "bold 10px monospace";
          ctx.fillText(`${tac.separation_distance ? tac.separation_distance.toFixed(1) : '---'}m`, mx + 6, my - 4);
        }
      }

      requestAnimationFrame(drawRadar);
    }
    requestAnimationFrame(drawRadar);

    // --- Chart.js Setup ---
    const ctxDist = document.getElementById("chartDistance").getContext("2d");
    const chartDist = new Chart(ctxDist, {
      type: "line",
      data: {
        labels: [],
        datasets: [{
          label: "Ayrılma Mesafesi (m)",
          data: [],
          borderColor: "#00e5ff",
          backgroundColor: "rgba(0, 229, 255, 0.1)",
          borderWidth: 2,
          pointRadius: 0,
          fill: true,
          tension: 0.2
        }]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: { display: false },
          y: { grid: { color: "rgba(255,255,255,0.05)" }, ticks: { color: "#94a3b8", font: { family: "monospace", size: 10 } } }
        },
        plugins: { legend: { display: false } }
      }
    });

    const ctxSpd = document.getElementById("chartSpeed").getContext("2d");
    const chartSpd = new Chart(ctxSpd, {
      type: "line",
      data: {
        labels: [],
        datasets: [
          {
            label: "Önleyici Hızı",
            data: [],
            borderColor: "#00e5ff",
            borderWidth: 2,
            pointRadius: 0,
            tension: 0.2
          },
          {
            label: "Talon Hızı",
            data: [],
            borderColor: "#ff3b30",
            borderWidth: 2,
            pointRadius: 0,
            tension: 0.2
          }
        ]
      },
      options: {
        responsive: true,
        maintainAspectRatio: false,
        animation: false,
        scales: {
          x: { display: false },
          y: { grid: { color: "rgba(255,255,255,0.05)" }, ticks: { color: "#94a3b8", font: { family: "monospace", size: 10 } } }
        },
        plugins: {
          legend: { labels: { color: "#cbd5e1", font: { family: "monospace", size: 10 } } }
        }
      }
    });

    function updateCharts(c) {
      chartDist.data.labels = c.times;
      chartDist.data.datasets[0].data = c.dist;
      chartDist.update("none");

      chartSpd.data.labels = c.times;
      chartSpd.data.datasets[0].data = c.v_int;
      chartSpd.data.datasets[1].data = c.v_tgt;
      chartSpd.update("none");
    }

    // Start WebSocket
    connectWebSocket();
  </script>
</body>
</html>
"""

@app.get("/", response_class=HTMLResponse)
async def get_index():
    return HTMLResponse(content=HTML_CONTENT)


# ----------------------------------------------------------------- Main ---

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=8080, help="Web GCS HTTP/WS port")
    args = parser.parse_args()

    # Start background threads
    threading.Thread(target=gz_transport_worker_thread, daemon=True).start()
    threading.Thread(target=mavlink_worker_thread, daemon=True).start()
    threading.Thread(target=target_poller_thread, daemon=True).start()
    threading.Thread(target=tactical_loop_thread, daemon=True).start()

    print(f"================================================================")
    print(f"  OCTOPUS TACTICAL WEB GCS STARTED!")
    print(f"  Tarayıcınızdan açın: http://localhost:{args.port}")
    print(f"================================================================", flush=True)

    uvicorn.run(app, host="0.0.0.0", port=args.port, log_level="warning")


if __name__ == "__main__":
    main()
