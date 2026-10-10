#!/usr/bin/env bash
#
# Start Gazebo, spawn the interceptor and the Talon 1718 target and attach a PX4
# SITL instance to the interceptor, which is flown from the ground station.
# The target has no PX4: tools/target_sim.py moves it along a square circuit and
# publishes its position at 2 Hz (HTTP JSON http://localhost:8000/target, and
# ADS-B through the interceptor PX4 to QGroundControl). tools/ground_relay.py plays the
# ground station: it polls that JSON and sends it to the interceptor as FOLLOW_TARGET
# (uORB follow_target).
#
# The interceptor nose camera is streamed as H.264 (udp 5600) and shown in its own
# window (tools/view.sh; QGroundControl's video source must stay disabled).
#
#   make px4_sitl                                   # once (and after airframe changes)
#   Tools/simulation/interceptor_sim/sim.sh         # GUI
#   HEADLESS=1 Tools/simulation/interceptor_sim/sim.sh
#
#   PX4 instance 0: interceptor, in the foreground (pxh> shell)
#                   QGroundControl: udp 14550 (MAV_SYS_ID 1), API/offboard: udp 14540
#
# Env:
#   WORLD             Gazebo world name (default: ankara, see tools/build_world.py)
#   TARGET_AUTO       0: do not start tools/target_sim.py (the target stays on the runway)
#   TARGET_ARGS       arguments for tools/target_sim.py, e.g. "--alt 60 --speed 18"
#   RELAY             0: do not start tools/ground_relay.py (no FOLLOW_TARGET to the interceptor)
#   DETECTOR          0: do not start tools/sim_detector.py (no vision detections)
#   TARGET_POSE       x,y,z,roll,pitch,yaw of the target spawn. The model origin is its
#                     CG, 0.12 m above the belly. Default: worlds/<WORLD>.env, else 0,0,0.15,0,0,0
#   HEADLESS          1: no Gazebo GUI and no camera window
#   VIEW              0: do not open the camera window
#   NVIDIA_OFFLOAD    0: do not force rendering on the NVIDIA GPU (hybrid graphics laptops)
#   CAMERA_STREAM_ENCODERS  encoder order for the camera streams, default auto
#                     (nvenc,nvcuda,vaapi,va,qsv,x264); CAMERA_STREAM_HOST / _PORT: destination
#   INTERCEPTOR       0 to start the target only
#   INTERCEPTOR_POSE  x,y,z,roll,pitch,yaw of the interceptor spawn. Default: next to the
#                     target (worlds/<WORLD>.env), height from models/interceptor/model.sdf

set -e

export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
BUILD_DIR="${PX4_DIR}/build/px4_sitl_default"
WORLD="${WORLD:-ankara}"
TARGET_POSE_DEFAULT="0,0,0.15,0,0,0"
# shellcheck disable=SC1090
[ -f "${SCRIPT_DIR}/worlds/${WORLD}.env" ] && . "${SCRIPT_DIR}/worlds/${WORLD}.env"
TARGET_POSE="${TARGET_POSE:-${TARGET_POSE_DEFAULT}}"
TARGET_INSTANCE=1
TARGET_NAME="talon1718_${TARGET_INSTANCE}"
INTERCEPTOR="${INTERCEPTOR:-1}"
INTERCEPTOR_INSTANCE=0
INTERCEPTOR_NAME="interceptor_${INTERCEPTOR_INSTANCE}"
INTERCEPTOR_SDF="${SCRIPT_DIR}/models/interceptor/model.sdf"

if [ ! -x "${BUILD_DIR}/bin/px4" ]; then
	echo "PX4 SITL not built, run: make px4_sitl" >&2
	exit 1
fi

if [ "${WORLD}" = "ankara" ] && [ ! -d "${SCRIPT_DIR}/models/terrain_mcmillan" ]; then
	echo "Terrain not generated, run: python3 ${SCRIPT_DIR}/tools/build_world.py" >&2
	exit 1
fi

if [ "${INTERCEPTOR}" != "0" ] && [ ! -f "${INTERCEPTOR_SDF}" ]; then
	echo "Interceptor model not generated, run: python3 ${SCRIPT_DIR}/tools/build_interceptor.py" >&2
	exit 1
fi

if ! command -v gz >/dev/null; then
	echo "Gazebo (gz) not found, install Gazebo Harmonic first" >&2
	exit 1
fi

# PX4 plugin/server config paths, then our models in front of the PX4 ones
# shellcheck disable=SC1091
. "${BUILD_DIR}/rootfs/gz_env.sh"
export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models:${SCRIPT_DIR}/worlds:${GZ_SIM_RESOURCE_PATH}"
# PX4's server.config, but with our camera stream plugin (udp 5600+, hardware H.264 when possible)
export GZ_SIM_SERVER_CONFIG_PATH="${SCRIPT_DIR}/gz/server.config"

# our Gazebo plugins (plugins/*): build when missing or changed
for plugin_dir in "${SCRIPT_DIR}"/plugins/*/; do
	name=$(basename "${plugin_dir}")
	plugin_build="${SCRIPT_DIR}/build/plugins/${name}"
	lib=$(find "${plugin_build}" -maxdepth 1 -name 'lib*.so' 2>/dev/null | head -1)

	if [ -z "${lib}" ] || [ -n "$(find "${plugin_dir}" -newer "${lib}" -type f)" ]; then
		echo "Building the ${name} plugin"
		cmake -S "${plugin_dir}" -B "${plugin_build}" -DCMAKE_BUILD_TYPE=Release > /dev/null
		cmake --build "${plugin_build}" -j"$(nproc)" > /dev/null
	fi

	export GZ_SIM_SYSTEM_PLUGIN_PATH="${plugin_build}:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
done

# Hybrid graphics (NVIDIA PRIME on-demand): render the Gazebo GUI and the camera sensors on the
# NVIDIA GPU instead of the integrated one
if [ "${NVIDIA_OFFLOAD:-1}" != "0" ] && command -v nvidia-smi >/dev/null && nvidia-smi -L >/dev/null 2>&1; then
	export __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia
	[ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ] && \
		export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
	echo "Rendering on the NVIDIA GPU (NVIDIA_OFFLOAD=0 to disable)"
fi
export GZ_IP=127.0.0.1

kill_tree() { # pid: the process and all its descendants
	local child

	for child in $(pgrep -P "$1"); do
		kill_tree "${child}"
	done

	kill "$1" 2>/dev/null || true
}

# background processes get a signal from the kernel when this script dies, also when it is
# killed with SIGKILL and the cleanup trap below cannot run (else they keep running).
# gz sim gets KILL: it does not exit on TERM once its parent is gone
PDEATH="setpriv --pdeathsig TERM --"
PDEATH_GZ="setpriv --pdeathsig KILL --"

# every process started from here inherits this; gz sim detaches from its parent, so
# the cleanup finds its server and GUI by this variable instead of the process tree
export INTERCEPTOR_SIM_ID=$$

cleanup() {
	local p

	for p in $(pgrep -P $$); do
		kill_tree "${p}"
	done

	for p in $(pgrep -f "gz sim"); do
		tr '\0' '\n' < "/proc/${p}/environ" 2>/dev/null | grep -qx "INTERCEPTOR_SIM_ID=$$" && kill_tree "${p}"
	done
}
trap cleanup EXIT

world_file="${SCRIPT_DIR}/worlds/${WORLD}.sdf"
[ -f "${world_file}" ] || world_file="${PX4_GZ_WORLDS}/${WORLD}.sdf"

# Stop what a previous run left behind: a second server in the same partition mixes up the
# GUI (empty scene), PX4 instances hold their instance lock and ports, target_sim.py port 8000.
stop_previous() {
	local p pids=() part

	for p in $(pgrep -f "^gz sim"); do
		part=$(tr '\0' '\n' < "/proc/${p}/environ" 2>/dev/null | sed -n 's/^GZ_PARTITION=//p')
		[ "${part}" = "${GZ_PARTITION:-}" ] && pids+=("${p}")
	done

	for p in $(pgrep -f "^${BUILD_DIR}/bin/px4 -i") $(pgrep -f "^python3 (-u )?${SCRIPT_DIR}/tools/(target_sim|ground_relay|sim_detector|view_hud).py"); do
		pids+=("${p}")
	done

	[ ${#pids[@]} -eq 0 ] && return

	echo "Stopping processes of a previous simulation: ${pids[*]}"

	for p in "${pids[@]}"; do
		kill_tree "${p}"
	done

	# up to 5 s to exit, then force
	for _ in $(seq 20); do
		local alive=0

		for p in "${pids[@]}"; do
			kill -0 "${p}" 2>/dev/null && alive=1
		done

		[ "${alive}" = 0 ] && return
		sleep 0.25
	done

	for p in "${pids[@]}"; do
		kill -9 "${p}" 2>/dev/null || true
	done
}

stop_previous

echo "Starting Gazebo world ${world_file}"
${PDEATH_GZ} gz sim --verbose=1 -r -s "${world_file}" &

if [ -z "${HEADLESS}" ]; then
	${PDEATH_GZ} gz sim -g >/dev/null 2>&1 &
fi

for _ in $(seq 30); do
	gz service -i --service "/world/${WORLD}/scene/info" 2>&1 | grep -q "Service providers" && break
	sleep 1
done

spawn() { # name model x,y,z,r,p,y [extra sdf inside the include, e.g. a plugin]
	IFS=, read -r x y z R P Y <<< "$3"
	gz service -s "/world/${WORLD}/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
		--timeout 5000 --req "name: \"$1\", allow_renaming: false, sdf: '<sdf version=\"1.9\"><include><uri>model://$2</uri><pose>${x} ${y} ${z:-0} ${R:-0} ${P:-0} ${Y:-0}</pose>${4:-}</include></sdf>'"
}

echo "Spawning ${TARGET_NAME} at ${TARGET_POSE}"
# kinematic target: moved by tools/target_sim.py through /model/<name>/cmd_vel
spawn "${TARGET_NAME}" talon1718 "${TARGET_POSE}" \
	'<plugin filename=\"gz-sim-velocity-control-system\" name=\"gz::sim::systems::VelocityControl\"/>'

if [ "${TARGET_AUTO:-1}" != "0" ]; then
	mkdir -p "${SCRIPT_DIR}/build"
	# shellcheck disable=SC2086
	${PDEATH} python3 -u "${SCRIPT_DIR}/tools/target_sim.py" --world "${WORLD}" --model "${TARGET_NAME}" ${TARGET_ARGS} \
		> "${SCRIPT_DIR}/build/target_sim.log" 2>&1 &
	echo "Target: tools/target_sim.py in the background, log: ${SCRIPT_DIR}/build/target_sim.log"
	echo "        position: http://localhost:8000/target (2 Hz), ADS-B in QGroundControl"

	if [ "${RELAY:-1}" != "0" ] && [ "${INTERCEPTOR}" != "0" ]; then
		${PDEATH} python3 -u "${SCRIPT_DIR}/tools/ground_relay.py" > "${SCRIPT_DIR}/build/ground_relay.log" 2>&1 &
		echo "Relay:  tools/ground_relay.py, FOLLOW_TARGET to the interceptor, log: ${SCRIPT_DIR}/build/ground_relay.log"
	fi

	if [ "${DETECTOR:-1}" != "0" ] && [ "${INTERCEPTOR}" != "0" ]; then
		${PDEATH} python3 -u "${SCRIPT_DIR}/tools/sim_detector.py" --world "${WORLD}" \
			--interceptor "${INTERCEPTOR_NAME}" --target "${TARGET_NAME}" \
			> "${SCRIPT_DIR}/build/sim_detector.log" 2>&1 &
		echo "Detector: tools/sim_detector.py, ground-truth vision → udp 15600, log: ${SCRIPT_DIR}/build/sim_detector.log"
	fi
fi

if [ "${INTERCEPTOR}" != "0" ]; then
	if [ -z "${INTERCEPTOR_POSE}" ]; then
		# the model origin is its CG: spawn it standing on its tail
		z=$(sed -n 's|^    <pose>0 0 \([0-9.]*\) 0 0 0</pose>|\1|p' "${INTERCEPTOR_SDF}" | head -1)
		IFS=, read -r ix iy iyaw <<< "${INTERCEPTOR_XY_DEFAULT:-0,0,0}"
		INTERCEPTOR_POSE="${ix},${iy},${z:-0.3},0,0,${iyaw}"
	fi

	echo "Spawning ${INTERCEPTOR_NAME} at ${INTERCEPTOR_POSE}"
	spawn "${INTERCEPTOR_NAME}" interceptor "${INTERCEPTOR_POSE}"
fi

if [ "${VIEW:-1}" != "0" ] && [ -z "${HEADLESS}" ]; then
	${PDEATH} "${SCRIPT_DIR}/tools/view.sh" --world "${WORLD}" --interceptor "${INTERCEPTOR_NAME}" --target "${TARGET_NAME}" > /dev/null 2>&1 &
fi

# Track the interceptor in the Gazebo GUI camera from the upper right rear
if [ -z "${HEADLESS}" ] && [ "${INTERCEPTOR}" != "0" ]; then
	(
		sleep 2.5
		"${SCRIPT_DIR}/tools/cam.sh" rear-right 4 "${INTERCEPTOR_NAME}" >/dev/null 2>&1 || true
		sleep 2.5
		"${SCRIPT_DIR}/tools/cam.sh" rear-right 4 "${INTERCEPTOR_NAME}" >/dev/null 2>&1 || true
	) &
fi

start_px4() { # instance autostart model_name [px4 options]
	mkdir -p "${BUILD_DIR}/instance_$1"
	cd "${BUILD_DIR}/instance_$1"
	PX4_SYS_AUTOSTART="$2" PX4_GZ_STANDALONE=1 PX4_GZ_WORLD="${WORLD}" PX4_GZ_MODEL_NAME="$3" \
		"${BUILD_DIR}/bin/px4" -i "$1" "${@:4}" "${BUILD_DIR}/etc"
}

if [ "${INTERCEPTOR}" != "0" ]; then
	# interceptor: PX4 in the foreground (pxh> shell)
	start_px4 "${INTERCEPTOR_INSTANCE}" 4051 "${INTERCEPTOR_NAME}"
else
	wait  # keep Gazebo running
fi
