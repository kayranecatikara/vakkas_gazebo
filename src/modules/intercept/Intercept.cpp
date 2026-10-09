/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "Intercept.hpp"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/posix.h>

Intercept::Intercept() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

Intercept::~Intercept()
{
	if (_sent_mode_registration) {
		UnregisterFlightMode();
	}
}

bool Intercept::init()
{
	ScheduleOnInterval(10_ms); // 100 Hz guidance loop
	return true;
}

void Intercept::RegisterFlightMode()
{
	register_ext_component_request_s req{};
	req.timestamp = hrt_absolute_time();
	strncpy(req.name, "Intercept", sizeof(req.name) - 1);
	req.request_id = _mode_request_id;
	req.px4_ros2_api_version = 1;
	req.register_arming_check = true;
	req.register_mode = true;
	req.register_mode_executor = false;
	req.enable_replace_internal_mode = false;
	req.activate_mode_immediately = false;
	_register_ext_component_request_pub.publish(req);
}

void Intercept::UnregisterFlightMode()
{
	unregister_ext_component_s unregister{};
	unregister.timestamp = hrt_absolute_time();
	strncpy(unregister.name, "Intercept", sizeof(unregister.name) - 1);
	unregister.arming_check_id = _arming_check_id;
	unregister.mode_id = _mode_id;
	unregister.mode_executor_id = -1;
	_unregister_ext_component_pub.publish(unregister);
}

void Intercept::CheckModeRegistration()
{
	register_ext_component_reply_s reply;
	int tries = register_ext_component_reply_s::ORB_QUEUE_LENGTH;

	while (_register_ext_component_reply_sub.update(&reply) && --tries >= 0) {
		if (reply.request_id == _mode_request_id && reply.success) {
			_arming_check_id = reply.arming_check_id;
			_mode_id = reply.mode_id;
			PX4_INFO("Intercept mode registered: arming_check_id=%d, mode_id=%d", _arming_check_id, _mode_id);
			break;
		}
	}
}

void Intercept::ReplyToArmingCheck(uint8_t request_id)
{
	arming_check_reply_s reply{};
	reply.timestamp = hrt_absolute_time();
	reply.request_id = request_id;
	reply.registration_id = _arming_check_id;
	reply.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
	reply.num_events = 0;
	reply.can_arm_and_run = true;
	reply.mode_req_angular_velocity = false;
	reply.mode_req_attitude = true;
	reply.mode_req_local_alt = true;
	reply.mode_req_local_position = true;
	reply.mode_req_local_position_relaxed = false;
	reply.mode_req_global_position = false;
	reply.mode_req_global_position_relaxed = false;
	reply.mode_req_mission = false;
	reply.mode_req_home_position = false;
	reply.mode_req_prevent_arming = false;
	reply.mode_req_manual_control = false;
	_arming_check_reply_pub.publish(reply);
}

void Intercept::UpdateTarget()
{
	// 1. Maintain local map projection reference from local position
	if (_local_pos.xy_global && _local_pos.z_global) {
		if (!_map_ref.isInitialized()
		    || _map_ref.getProjectionReferenceTimestamp() != _local_pos.ref_timestamp) {
			_map_ref.initReference(_local_pos.ref_lat, _local_pos.ref_lon, _local_pos.ref_timestamp);
		}
	}

	// 2. Read new follow_target messages from relay
	if (_follow_target_sub.updated()) {
		follow_target_s ft;

		if (_follow_target_sub.copy(&ft)) {
			const double target_lat = ft.lat;
			const double target_lon = ft.lon;

			if (_map_ref.isInitialized() && PX4_ISFINITE(target_lat) && PX4_ISFINITE(target_lon)) {
				float x{0.f};
				float y{0.f};
				_map_ref.project(target_lat, target_lon, x, y);

				_target_pos(0) = x;
				_target_pos(1) = y;
				_target_pos(2) = -(ft.alt - _local_pos.ref_alt);

				_target_vel(0) = ft.vx;
				_target_vel(1) = ft.vy;
				_target_vel(2) = ft.vz;

				_target_pos_valid = true;
				_last_target_update = hrt_absolute_time();
			}
		}
	}

	// 3. Invalidate target if no update for more than 3 seconds
	if (_target_pos_valid && (hrt_elapsed_time(&_last_target_update) > 3_s)) {
		_target_pos_valid = false;
	}
}

void Intercept::UpdateVisualDetection()
{
	target_detection_s td;

	_has_fresh_visual = false;

	if (_target_detection_sub.update(&td)) {
		if (td.detected) {
			if (!_visual_contact) {
				PX4_INFO("Visual contact acquired! Range: %.1f m, bbox: [%.2f, %.2f, %.2f, %.2f]",
					 (double)td.range_m, (double)td.bbox[0], (double)td.bbox[1],
					 (double)td.bbox[2], (double)td.bbox[3]);
			}

			_visual_contact = true;
			_visual_range = td.range_m;
			_target_detection = td;
			_last_visual_contact = hrt_absolute_time();
			_has_fresh_visual = true;
		}
	}

	const float lost_timeout_s = math::max(_param_int_lost_timeout.get(), 0.2f);
	const uint64_t lost_timeout_us = (uint64_t)(lost_timeout_s * 1e6f);

	if (_visual_contact && (hrt_elapsed_time(&_last_visual_contact) > lost_timeout_us)) {
		PX4_WARN("Visual contact lost (>%.1fs)", (double)lost_timeout_s);
		_visual_contact = false;
	}
}

void Intercept::ComputeApproachGuidance(matrix::Vector3f &vel_cmd, float &yaw_cmd)
{
	const hrt_abstime now = hrt_absolute_time();
	float dt = 0.02f; // default 50 Hz work item
	if (_last_approach_time > 0 && now > _last_approach_time) {
		dt = math::constrain((float)(now - _last_approach_time) * 1e-6f, 0.005f, 0.1f);
	}
	_last_approach_time = now;

	const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
	const float dist_to_target = (_target_pos - self_pos).norm();

	// 1. Closing rate calculation (Yol B: mesafe kapanma hızı)
	if (_last_approach_dist < 0.f) {
		_last_approach_dist = dist_to_target;
		_filt_closing_speed = 0.f;
	} else if (dt > 0.005f) {
		const float raw_closing_speed = (_last_approach_dist - dist_to_target) / dt;
		_filt_closing_speed = 0.85f * _filt_closing_speed + 0.15f * raw_closing_speed;
		_last_approach_dist = dist_to_target;
	}

	// Initialize continuous speed from current forward speed if needed
	const float psi = _local_pos.heading;
	const float current_fwd_speed = _local_pos.vx * cosf(psi) + _local_pos.vy * sinf(psi);
	if (!_speed_initialized) {
		_fwd_speed_cmd = math::constrain(current_fwd_speed, 20.0f, 45.0f);
		_fwd_accel_cmd = 0.0f;
		_speed_initialized = true;
	}

	// Target flight direction and speed
	const float target_speed = _target_vel.norm();
	matrix::Vector3f target_dir(1.f, 0.f, 0.f);

	if (target_speed > 1.f) {
		target_dir = _target_vel / target_speed;
	}

	const float min_closing = _param_int_min_closing.get(); // default 5.0 m/s

	// 2. Direct acceleration control based on closing distance and closing rate
	if (dist_to_target > 100.0f) {
		// Far away (>100m): full throttle pursuit (+5.0 m/s^2 towards 45 m/s top speed)
		_fwd_accel_cmd = 5.0f;
	} else {
		// Inside 100m approaching visual range (50m):
		// Desired closing speed tapers from ~16 m/s at 100m down to min_closing (8 m/s) at 50m
		const float desired_closing = min_closing + (dist_to_target - 50.0f) * 0.16f;
		const float closing_err = _filt_closing_speed - desired_closing;

		if (closing_err > 0.0f) {
			// Closing faster than desired: smooth braking
			_fwd_accel_cmd = math::constrain(-1.5f - 0.3f * closing_err, -5.0f, -0.8f);
		} else if (closing_err < -2.0f) {
			// Closing too slowly: gentle positive acceleration
			_fwd_accel_cmd = 2.0f;
		} else {
			_fwd_accel_cmd = 0.0f;
		}
	}

	// Integrate continuous speed from acceleration command
	_fwd_speed_cmd += _fwd_accel_cmd * dt;

	// Clamp forward speed:
	// Minimum speed GUARANTEES closing speed never drops below min_closing (default 8 m/s)!
	const float min_approach_speed = _target_pos_valid ? math::max(target_speed + min_closing, 15.0f) : 15.0f;
	_fwd_speed_cmd = math::constrain(_fwd_speed_cmd, min_approach_speed, 45.0f);

	// 3. Aim point placed 30m directly behind target along its flight line
	const float dist_behind = 30.f;
	const matrix::Vector3f aim_point = _target_pos - target_dir * dist_behind;

	const matrix::Vector3f to_aim = aim_point - self_pos;
	const float dist_to_aim = to_aim.norm();
	const float time_to_aim = dist_to_aim / math::max(_fwd_speed_cmd, 10.0f);

	const matrix::Vector3f intercept_point = aim_point + _target_vel * time_to_aim;
	const matrix::Vector3f to_intercept = intercept_point - self_pos;

	// 4. Horizontal velocity setpoint in inertial NED
	const matrix::Vector2f to_intercept_xy(to_intercept(0), to_intercept(1));
	const float dist_xy = to_intercept_xy.norm();
	float desired_vx{0.f};
	float desired_vy{0.f};

	if (dist_xy > 0.5f) {
		const matrix::Vector2f vel_xy = (to_intercept_xy / dist_xy) * _fwd_speed_cmd;
		desired_vx = vel_xy(0);
		desired_vy = vel_xy(1);
	} else {
		desired_vx = target_dir(0) * _fwd_speed_cmd;
		desired_vy = target_dir(1) * _fwd_speed_cmd;
	}

	// Horizontal acceleration slew-rate limit matching MPC_ACC_HOR
	if (_last_cmd_valid && dt > 0.005f) {
		const float max_acc_hor = _param_mpc_acc_hor.get();
		const float max_dvh = max_acc_hor * dt;
		vel_cmd(0) = math::constrain(desired_vx, _last_vel_cmd(0) - max_dvh, _last_vel_cmd(0) + max_dvh);
		vel_cmd(1) = math::constrain(desired_vy, _last_vel_cmd(1) - max_dvh, _last_vel_cmd(1) + max_dvh);
	} else {
		vel_cmd(0) = desired_vx;
		vel_cmd(1) = desired_vy;
	}

	// 5. Altitude controller in Approach mode
	// delta_z in NED: negative means intercept point is higher (climb), positive is lower (descend)
	const float delta_z = intercept_point(2) - self_pos(2);
	const float target_vz = _target_vel(2);
	const float K_z_p = _param_int_kp_z_app.get();
	const float vz_desired = math::constrain(target_vz + K_z_p * delta_z, -7.5f, 3.5f);

	// Slew-rate limit vertical velocity (max 5.0 m/s^2 acceleration) to rapidly match altitude
	if (_last_cmd_valid && dt > 0.005f) {
		const float max_dvz = 5.0f * dt;
		vel_cmd(2) = math::constrain(vz_desired, _last_vel_cmd(2) - max_dvz, _last_vel_cmd(2) + max_dvz);
	} else {
		vel_cmd(2) = vz_desired;
	}

	// 6. Yaw setpoint: point drone nose horizontally towards target position with slew-rate limiting
	const float dx = _target_pos(0) - _local_pos.x;
	const float dy = _target_pos(1) - _local_pos.y;
	const float desired_yaw = atan2f(dy, dx);

	if (_last_cmd_valid && dt > 0.005f) {
		const float max_yaw_rate = math::radians(_param_int_yaw_rate.get());
		const float max_dyaw = max_yaw_rate * dt;
		const float yaw_err = matrix::wrap_pi(desired_yaw - _last_yaw_cmd);
		yaw_cmd = matrix::wrap_pi(_last_yaw_cmd + math::constrain(yaw_err, -max_dyaw, max_dyaw));
	} else {
		yaw_cmd = desired_yaw;
	}
}

void Intercept::ComputeClosePursuitGuidance(matrix::Vector3f &vel_cmd, float &yaw_cmd)
{
	const hrt_abstime now = hrt_absolute_time();

	const float cx = _target_detection.bbox[0]; // [0.0, 1.0], center is 0.50
	const float cy = _target_detection.bbox[1]; // [0.0, 1.0], center is 0.50
	const float w  = _target_detection.bbox[2]; // Target bounding box width fraction

	// Desired visual size setpoint
	const float desired_w = math::constrain(_param_int_tgt_size.get(), 0.10f, 0.60f);

	// Compute time step for optical derivative estimation
	float dt = 0.033f; // default 30 Hz
	if (_last_visual_time > 0 && now > _last_visual_time) {
		dt = math::constrain((float)(now - _last_visual_time) * 1e-6f, 0.005f, 0.15f);
	}
	_last_visual_time = now;

	// 1. 3D Line-of-Sight in Inertial NED frame
	const matrix::Dcmf R_nb(matrix::Quatf(_vehicle_attitude.q));
	const matrix::Vector3f los_body(_target_detection.los_body);
	matrix::Vector3f los_ned = R_nb * los_body;

	if (los_ned.longerThan(FLT_EPSILON)) {
		los_ned.normalize();
	} else {
		los_ned = matrix::Vector3f(1.f, 0.f, 0.f);
	}

	_last_los_ned = los_ned;
	_los_ned_valid = true;

	// Current forward ground speed along heading
	const float psi = _local_pos.heading;
	const float current_fwd_speed = _local_pos.vx * cosf(psi) + _local_pos.vy * sinf(psi);

	// Initialize continuous forward speed if needed
	if (!_speed_initialized || _fwd_speed_cmd < 5.0f) {
		_fwd_speed_cmd = math::constrain(current_fwd_speed, 18.0f, 45.0f);
		_fwd_accel_cmd = 0.0f;
		_speed_initialized = true;
	}

	// 2. Optical Looming Divergence & Self-Speed Referenced Standoff Velocity Control
	// Estimate bounding box expansion rate (dw/dt)
	const float raw_d_w_dt = (w - _last_w) / dt;
	_filt_d_w_dt = 0.80f * _filt_d_w_dt + 0.20f * raw_d_w_dt;

	const float w_safe = math::max(w, 0.02f);
	const float divergence = _filt_d_w_dt / w_safe; // (dw/dt)/w = V_rel / Dist (1/s)

	// Optically estimated relative closing velocity: V_rel = Dist * divergence ~ (1.5 / w) * divergence (m/s)
	// (Positive = closing in, Negative = target pulling away)
	const float v_rel = math::constrain((1.5f / w_safe) * divergence, -10.0f, 10.0f);

	// Bounding box size tolerance corridor (+/- 0.04 around desired_w, e.g. [0.46, 0.54] for desired_w = 0.50)
	constexpr float W_TOL = 0.04f;
	const float w_min = desired_w - W_TOL;
	const float w_max = desired_w + W_TOL;

	const float kp_opt = _param_int_kp_opt.get();
	const float kd_opt = _param_int_kd_opt.get();
	const float max_acc_hor = _param_mpc_acc_hor.get();

	float delta_v = 0.0f;

	if (w < w_min) {
		// Target too far (w < 0.46): drive towards corridor with PD damping
		const float norm_err_w = (w_min - w) / desired_w;
		delta_v = kp_opt * norm_err_w - kd_opt * v_rel;

	} else if (w > w_max) {
		// Target too close (w > 0.54): decelerate back into tolerance corridor
		const float norm_err_w = (w_max - w) / desired_w; // negative
		delta_v = kp_opt * norm_err_w - kd_opt * v_rel;

	} else {
		// Target inside [0.46, 0.54] tolerance corridor (~3m +/- 24cm):
		// Standoff distance is satisfied. Damp residual relative velocity to lock onto target speed!
		if (fabsf(v_rel) > 0.15f) {
			delta_v = -kd_opt * v_rel;
		} else {
			delta_v = 0.0f;
		}
	}

	// Adaptive maneuver corner braking:
	// If target drifts towards FOV edges beyond +/- 0.25 deadband ([0.25, 0.75]),
	// gently brake forward speed proportionally to (size * lateral_drift) to tighten turn radius
	const float lat_offset = fabsf(cx - 0.50f);
	constexpr float LAT_BRAKE_DEADBAND = 0.25f; // +/- 0.25 deadband corridor ([0.25, 0.75])
	float corner_brake = 0.0f;
	if (lat_offset > LAT_BRAKE_DEADBAND) {
		const float lat_drift = lat_offset - LAT_BRAKE_DEADBAND;
		const float k_corner = _param_int_k_corner.get();
		corner_brake = math::constrain(k_corner * w * lat_drift, 0.0f, 3.0f);
	}
	delta_v -= corner_brake;

	// Clamp commanded velocity delta to prevent abrupt pitch swings
	delta_v = math::constrain(delta_v, -5.0f, 5.0f);

	// Desired forward speed is referenced to vehicle's OWN actual ground speed
	// Multicopter has no stall speed: allow slowing all the way down to hover (0.0 m/s) if needed during sharp maneuvers!
	const float desired_fwd_speed = math::constrain(current_fwd_speed + delta_v, 0.0f, 45.0f);

	// Slew-rate limit forward speed command matching MPC_ACC_HOR for smooth aerodynamic pitch transitions
	const float prev_speed_cmd = _fwd_speed_cmd;
	if (_last_cmd_valid && dt > 0.005f) {
		const float max_dv = max_acc_hor * dt;
		_fwd_speed_cmd = math::constrain(desired_fwd_speed, _fwd_speed_cmd - max_dv, _fwd_speed_cmd + max_dv);
		_fwd_accel_cmd = (_fwd_speed_cmd - prev_speed_cmd) / dt;
	} else {
		_fwd_speed_cmd = desired_fwd_speed;
		_fwd_accel_cmd = 0.0f;
	}

	_last_w = w;

	// 3. 3D Velocity Command
	// Longitudinal horizontal velocity along line of sight in XY plane
	const float los_xy_norm = sqrtf(los_ned(0) * los_ned(0) + los_ned(1) * los_ned(1));
	const float inv_los_xy = (los_xy_norm > 0.01f) ? (1.0f / los_xy_norm) : 1.0f;
	const float los_x_norm = los_ned(0) * inv_los_xy;
	const float los_y_norm = los_ned(1) * inv_los_xy;

	// Lateral visual centering: deadband corridor +/- 0.04 around screen center (0.50)
	const float kp_lat = _param_int_kp_lat.get();
	float lat_err = 0.0f;
	if (cx < 0.46f) {
		lat_err = cx - 0.46f;
	} else if (cx > 0.54f) {
		lat_err = cx - 0.54f;
	} else {
		lat_err = 0.0f;
	}
	const float vel_lat_corr = math::constrain(lat_err * kp_lat, -8.0f, 8.0f);

	// Unit horizontal vector perpendicular to LOS in XY plane (pointing right)
	const float right_x = -los_y_norm;
	const float right_y =  los_x_norm;

	float desired_vx = los_x_norm * _fwd_speed_cmd + right_x * vel_lat_corr;
	float desired_vy = los_y_norm * _fwd_speed_cmd + right_y * vel_lat_corr;

	// Slew-rate limit horizontal acceleration matching MPC_ACC_HOR
	if (_last_cmd_valid && dt > 0.005f) {
		const float max_dvh = max_acc_hor * dt;
		vel_cmd(0) = math::constrain(desired_vx, _last_vel_cmd(0) - max_dvh, _last_vel_cmd(0) + max_dvh);
		vel_cmd(1) = math::constrain(desired_vy, _last_vel_cmd(1) - max_dvh, _last_vel_cmd(1) + max_dvh);
	} else {
		vel_cmd(0) = desired_vx;
		vel_cmd(1) = desired_vy;
	}

	// Vertical altitude tracking with deadband corridor (+/- 0.15m)
	// Pure optical visual guidance: does NOT assume or require target GPS, barometer, or telemetry!
	// Computes physical altitude difference directly from optical line-of-sight elevation and estimated distance:
	const float est_dist = math::constrain(1.5f / w_safe, 1.0f, 50.0f);
	const float delta_z = est_dist * los_ned(2); // In NED: negative = target is higher (climb), positive = lower

	constexpr float Z_TOL = 0.15f; // +/- 15 cm deadband corridor
	float delta_z_corr = 0.0f;
	if (delta_z < -Z_TOL) {
		delta_z_corr = delta_z + Z_TOL; // target higher (NED negative) -> climb
	} else if (delta_z > Z_TOL) {
		delta_z_corr = delta_z - Z_TOL; // target lower (NED positive) -> descend
	} else {
		delta_z_corr = 0.0f; // within +/- 15cm corridor -> hold level flight!
	}

	const float kp_z = _param_int_kp_z.get();
	const float vz_desired = math::constrain(delta_z_corr * kp_z, -2.5f, 2.0f);

	// Slew-rate limit vertical acceleration (max 1.5 m/s^2) to prevent pitch oscillations
	if (_last_cmd_valid && dt > 0.005f) {
		const float max_dvz = 1.5f * dt;
		vel_cmd(2) = math::constrain(vz_desired, _last_vel_cmd(2) - max_dvz, _last_vel_cmd(2) + max_dvz);
	} else {
		vel_cmd(2) = vz_desired;
	}

	// 4. Active Seeker Yaw: point nose directly at target azimuth in world NED frame
	// Keeps target optical axis locked in the center of the camera horizontal FOV (+/- 30 deg)
	float desired_yaw = _last_cmd_valid ? _last_yaw_cmd : _local_pos.heading;
	if (los_xy_norm > 0.05f) {
		desired_yaw = atan2f(los_ned(1), los_ned(0));
	}

	if (_last_cmd_valid && dt > 0.005f) {
		const float max_yaw_rate = math::radians(_param_int_yaw_rate.get());
		const float max_dyaw = max_yaw_rate * dt;
		const float yaw_err = matrix::wrap_pi(desired_yaw - _last_yaw_cmd);
		yaw_cmd = matrix::wrap_pi(_last_yaw_cmd + math::constrain(yaw_err, -max_dyaw, max_dyaw));
	} else {
		yaw_cmd = desired_yaw;
	}

	_last_cx = cx;
	_last_cy = cy;
	_last_w  = w;
}

void Intercept::Run()
{
	if (should_exit()) {
		ScheduleClear();

		if (_sent_mode_registration) {
			UnregisterFlightMode();
		}

		exit_and_cleanup();
		return;
	}

	// 1. Parameter update
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// 2. Register flight mode with Commander
	if (!_sent_mode_registration) {
		RegisterFlightMode();
		_sent_mode_registration = true;
		return;
	}

	// 3. Check for registration confirmation
	if (_mode_id == -1 || _arming_check_id == -1) {
		CheckModeRegistration();
		return;
	}

	// 4. Respond to arming checks from Commander
	if (_arming_check_request_sub.updated()) {
		arming_check_request_s req;
		_arming_check_request_sub.copy(&req);
		ReplyToArmingCheck(req.request_id);
	}

	// 5. Update local position, attitude, target tracking, and visual detection
	_vehicle_local_position_sub.update(&_local_pos);
	_vehicle_attitude_sub.update(&_vehicle_attitude);
	UpdateTarget();
	UpdateVisualDetection();

	// 6. Check if Intercept is currently the active navigation mode
	vehicle_status_s vehicle_status;

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_is_active = (vehicle_status.nav_state == _mode_id);
	}

	// 7. Mode execution
	if (_is_active) {
		if (!_was_active) {
			PX4_INFO("Intercept mode activated: pursuing target");
			_was_active = true;
			_hold_valid = false;
			_guidance_state = GuidanceState::APPROACH;
			_last_cmd_valid = false;
			_last_visual_time = 0;
			_speed_initialized = false;
			_los_ned_valid = false;
			_fwd_speed_cmd = 45.f;
			_fwd_accel_cmd = 0.f;
			_speed_initialized = false;
			_last_approach_dist = -1.f;
			_filt_closing_speed = 0.f;
			_last_approach_time = 0;
		}

		// State transitions: APPROACH <-> CLOSE_PURSUIT
		if (_guidance_state == GuidanceState::APPROACH) {
			// Transition to CLOSE_PURSUIT as soon as target is acquired visually
			if (_visual_contact && _target_detection.detected && (_target_detection.bbox[2] > 0.005f)) {
				_guidance_state = GuidanceState::CLOSE_PURSUIT;
				_last_visual_time = 0;
				_los_ned_valid = false;
				_filt_d_w_dt = 0.f;

				_last_w = _target_detection.bbox[2];
				_last_cx = _target_detection.bbox[0];
				_last_cy = _target_detection.bbox[1];

				if (!_last_cmd_valid) {
					_last_yaw_cmd = _local_pos.heading;
					_last_vel_cmd(0) = _local_pos.vx;
					_last_vel_cmd(1) = _local_pos.vy;
					_last_vel_cmd(2) = _local_pos.vz;
					_last_cmd_valid = true;
				}

				PX4_INFO("Visual contact acquired! Handover to CLOSE_PURSUIT (IBVS), speed=%.1f m/s, bbox_w=%.3f",
					 (double)_fwd_speed_cmd, (double)_last_w);
			}

		} else if (_guidance_state == GuidanceState::CLOSE_PURSUIT) {
			if (!_visual_contact) {
				_guidance_state = GuidanceState::APPROACH;
				_last_cmd_valid = false;
				_last_visual_time = 0;
				_speed_initialized = false;
				_los_ned_valid = false;
				_last_approach_dist = -1.f;
				_last_approach_time = 0;
				PX4_WARN("Visual contact lost (>%.1fs), reverting to APPROACH", (double)_param_int_lost_timeout.get());
			}
		}

		if (_guidance_state == GuidanceState::CLOSE_PURSUIT) {
			matrix::Vector3f vel_cmd;
			float yaw_cmd{_local_pos.heading};

			if (_target_detection.detected && (_target_detection.bbox[2] > 0.005f)) {
				matrix::Vector3f prev_vel = _last_vel_cmd;
				float prev_yaw = _last_yaw_cmd;
				hrt_abstime now = hrt_absolute_time();

				ComputeClosePursuitGuidance(vel_cmd, yaw_cmd);

				// Compute command derivatives for intermediate 100 Hz predictive extrapolation
				if (_last_cmd_valid && _last_fresh_visual_time > 0) {
					float dt_frame = math::constrain((float)(now - _last_fresh_visual_time) * 1e-6f, 0.005f, 0.1f);
					_vel_cmd_dot = (vel_cmd - prev_vel) / dt_frame;
					_yaw_cmd_dot = matrix::wrap_pi(yaw_cmd - prev_yaw) / dt_frame;

					// Clamp derivative limits for safety (max 20 m/s^2 accel, max 3.0 rad/s yaw rate)
					for (int i = 0; i < 3; i++) {
						_vel_cmd_dot(i) = math::constrain(_vel_cmd_dot(i), -20.f, 20.f);
					}
					_yaw_cmd_dot = math::constrain(_yaw_cmd_dot, -3.0f, 3.0f);
				} else {
					_vel_cmd_dot.zero();
					_yaw_cmd_dot = 0.f;
				}

				_last_vel_cmd = vel_cmd;
				_last_yaw_cmd = yaw_cmd;
				_last_fresh_visual_time = now;
				_last_cmd_valid = true;

			} else if (_target_pos_valid) {
				// Target lost from camera frame:
				// Immediately turn drone head (yaw) towards target's current position to re-acquire!
				const float dx = _target_pos(0) - _local_pos.x;
				const float dy = _target_pos(1) - _local_pos.y;
				const float dist_xy = sqrtf(dx * dx + dy * dy);
				const float desired_yaw = atan2f(dy, dx);
				const float dt = 0.010f; // 100Hz work item step

				if (_last_cmd_valid) {
					const float max_yaw_rate = math::radians(_param_int_yaw_rate.get());
					const float max_dyaw = max_yaw_rate * dt;
					const float yaw_err = matrix::wrap_pi(desired_yaw - _last_yaw_cmd);
					yaw_cmd = matrix::wrap_pi(_last_yaw_cmd + math::constrain(yaw_err, -max_dyaw, max_dyaw));
				} else {
					yaw_cmd = desired_yaw;
				}

				// Fly towards target position with continuous speed
				const float pursuit_speed = math::constrain(_fwd_speed_cmd, 12.0f, 40.0f);
				if (dist_xy > 0.5f) {
					vel_cmd(0) = (dx / dist_xy) * pursuit_speed;
					vel_cmd(1) = (dy / dist_xy) * pursuit_speed;
				} else {
					vel_cmd(0) = _last_vel_cmd(0);
					vel_cmd(1) = _last_vel_cmd(1);
				}

				// Altitude: track target altitude
				const float dz = _target_pos(2) - _local_pos.z;
				vel_cmd(2) = math::constrain(dz * _param_int_kp_z.get(), -2.5f, 2.0f);

				_last_vel_cmd = vel_cmd;
				_last_yaw_cmd = yaw_cmd;
				_last_cmd_valid = true;

			} else if (_last_cmd_valid) {
				// Coast on last known command during single-frame dropouts
				hrt_abstime now = hrt_absolute_time();
				float tau = (float)(now - _last_fresh_visual_time) * 1e-6f;
				float damping = expf(-tau / 0.040f); // 40ms decay constant

				vel_cmd = _last_vel_cmd + (_vel_cmd_dot * tau) * damping;
				yaw_cmd = matrix::wrap_pi(_last_yaw_cmd + (_yaw_cmd_dot * tau) * damping);
			}

			trajectory_setpoint_s sp{};
			sp.timestamp = hrt_absolute_time();
			sp.position[0] = NAN;
			sp.position[1] = NAN;
			sp.position[2] = NAN;
			sp.velocity[0] = vel_cmd(0);
			sp.velocity[1] = vel_cmd(1);
			sp.velocity[2] = vel_cmd(2);
			sp.acceleration[0] = NAN;
			sp.acceleration[1] = NAN;
			sp.acceleration[2] = NAN;
			sp.yaw = yaw_cmd;
			sp.yawspeed = NAN;
			_trajectory_setpoint_pub.publish(sp);

		} else if (_target_pos_valid) {
			// Approach guidance: fly towards aim point behind target
			matrix::Vector3f vel_cmd;
			float yaw_cmd{_local_pos.heading};
			ComputeApproachGuidance(vel_cmd, yaw_cmd);
			_last_vel_cmd = vel_cmd;
			_last_yaw_cmd = yaw_cmd;
			_last_cmd_valid = true;

			trajectory_setpoint_s sp{};
			sp.timestamp = hrt_absolute_time();
			sp.position[0] = NAN;
			sp.position[1] = NAN;
			sp.position[2] = NAN;
			sp.velocity[0] = vel_cmd(0);
			sp.velocity[1] = vel_cmd(1);
			sp.velocity[2] = vel_cmd(2);
			sp.acceleration[0] = NAN;
			sp.acceleration[1] = NAN;
			sp.acceleration[2] = NAN;
			sp.yaw = yaw_cmd;
			sp.yawspeed = NAN;
			_trajectory_setpoint_pub.publish(sp);

		} else {
			// Fallback: if target GPS not yet valid, hold current position
			if (!_hold_valid && _local_pos.xy_valid && _local_pos.z_valid) {
				_hold_position(0) = _local_pos.x;
				_hold_position(1) = _local_pos.y;
				_hold_position(2) = _local_pos.z;
				_hold_yaw = _local_pos.heading;
				_hold_valid = true;
			}

			if (_hold_valid) {
				trajectory_setpoint_s sp{};
				sp.timestamp = hrt_absolute_time();
				sp.position[0] = _hold_position(0);
				sp.position[1] = _hold_position(1);
				sp.position[2] = _hold_position(2);
				sp.velocity[0] = 0.f;
				sp.velocity[1] = 0.f;
				sp.velocity[2] = 0.f;
				sp.acceleration[0] = NAN;
				sp.acceleration[1] = NAN;
				sp.acceleration[2] = NAN;
				sp.yaw = _hold_yaw;
				sp.yawspeed = 0.f;
				_trajectory_setpoint_pub.publish(sp);
			}
		}

	} else {
		if (_was_active) {
			PX4_INFO("Intercept mode deactivated");
			_was_active = false;
			_hold_valid = false;
			_guidance_state = GuidanceState::APPROACH;
			_last_cmd_valid = false;
			_last_visual_time = 0;
			_speed_initialized = false;
			_los_ned_valid = false;
			_fwd_speed_cmd = 45.f;
			_fwd_accel_cmd = 0.f;
			_last_approach_dist = -1.f;
			_filt_closing_speed = 0.f;
			_last_approach_time = 0;
		}
	}
}

int Intercept::print_status()
{
	PX4_INFO("Running: %s", is_running() ? "yes" : "no");
	PX4_INFO("Registered: %s (mode_id=%d, arming_check_id=%d)",
		 (_mode_id != -1) ? "yes" : "no", _mode_id, _arming_check_id);
	PX4_INFO("Active: %s", _is_active ? "yes" : "no");

	if (_hold_valid) {
		PX4_INFO("Holding position: [%.2f, %.2f, %.2f] m, yaw: %.1f deg",
			 (double)_hold_position(0), (double)_hold_position(1), (double)_hold_position(2),
			 (double)math::degrees(_hold_yaw));
	}

	if (_target_pos_valid) {
		const matrix::Vector3f self_pos(_local_pos.x, _local_pos.y, _local_pos.z);
		const matrix::Vector3f rel_pos = _target_pos - self_pos;
		const float dist = rel_pos.norm();
		const float age_s = (float)(hrt_elapsed_time(&_last_target_update)) * 1e-6f;

		PX4_INFO("Target GPS: [%.1f, %.1f, %.1f] m, vel: [%.1f, %.1f, %.1f] m/s, dist: %.1f m (age: %.2f s)",
			 (double)_target_pos(0), (double)_target_pos(1), (double)_target_pos(2),
			 (double)_target_vel(0), (double)_target_vel(1), (double)_target_vel(2),
			 (double)dist, (double)age_s);

	} else {
		PX4_INFO("Target GPS: no valid fix");
	}

	PX4_INFO("Guidance state: %s", (_guidance_state == GuidanceState::CLOSE_PURSUIT) ? "CLOSE_PURSUIT (IBVS Visual Servoing)" : "APPROACH (Midcourse GPS)");
	PX4_INFO("Speed setpoint: %.1f m/s, accel_cmd: %.2f m/s^2, closing rate: %.1f m/s",
		 (double)_fwd_speed_cmd, (double)_fwd_accel_cmd, (double)_filt_closing_speed);

	if (_visual_contact) {
		PX4_INFO("Visual contact: YES (range: %.1f m, bbox: [%.2f, %.2f, %.2f, %.2f])",
			 (double)_visual_range, (double)_target_detection.bbox[0], (double)_target_detection.bbox[1],
			 (double)_target_detection.bbox[2], (double)_target_detection.bbox[3]);
		PX4_INFO("Forward speed command: %.1f m/s, LOS NED: [%.2f, %.2f, %.2f]",
			 (double)_fwd_speed_cmd, (double)_last_los_ned(0), (double)_last_los_ned(1), (double)_last_los_ned(2));
	} else {
		PX4_INFO("Visual contact: NO");
	}

	return 0;
}

int Intercept::task_spawn(int argc, char *argv[])
{
	Intercept *instance = new Intercept();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

		PX4_ERR("init failed");

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int Intercept::custom_command(int argc, char *argv[])
{
	return print_usage("unrecognized command");
}

int Intercept::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Autonomous target intercept and close pursuit flight mode.
Two-stage guidance:
  1. APPROACH: Midcourse lead-pursuit towards target GPS position.
  2. CLOSE_PURSUIT: Pure IBVS visual servoing & 3D LOS station-keeping.
Registers external flight mode 'Intercept' with Commander.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("intercept", "mode");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int intercept_main(int argc, char *argv[])
{
	return Intercept::main(argc, argv);
}
