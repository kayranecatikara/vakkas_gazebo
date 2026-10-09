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

#pragma once

#include <lib/geo/geo.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/follow_target.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/target_detection.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class Intercept : public ModuleBase<Intercept>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	enum class GuidanceState : uint8_t {
		APPROACH = 0,       // Midcourse GPS approach towards target (lead pursuit to visual acquisition range)
		CLOSE_PURSUIT = 1   // Terminal visual servoing & station-keeping behind target (IBVS)
	};

	Intercept();
	~Intercept() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	void Run() override;

	void RegisterFlightMode();
	void UnregisterFlightMode();
	void CheckModeRegistration();
	void ReplyToArmingCheck(uint8_t request_id);
	void UpdateTarget();
	void UpdateVisualDetection();
	void ComputeApproachGuidance(matrix::Vector3f &vel_cmd, float &yaw_cmd);
	void ComputeClosePursuitGuidance(matrix::Vector3f &vel_cmd, float &yaw_cmd);

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _register_ext_component_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription _arming_check_request_sub{ORB_ID(arming_check_request)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _follow_target_sub{ORB_ID(follow_target)};
	uORB::Subscription _target_detection_sub{ORB_ID(target_detection)};

	// Publications
	uORB::Publication<register_ext_component_request_s> _register_ext_component_request_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s> _unregister_ext_component_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<arming_check_reply_s> _arming_check_reply_pub{ORB_ID(arming_check_reply)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};

	// Registration state
	bool _sent_mode_registration{false};
	const uint64_t _mode_request_id{849201};
	int8_t _arming_check_id{-1};
	int8_t _mode_id{-1};

	// Mode execution state
	bool _is_active{false};
	bool _was_active{false};
	bool _hold_valid{false};
	matrix::Vector3f _hold_position{};
	float _hold_yaw{0.f};

	GuidanceState _guidance_state{GuidanceState::APPROACH};
	vehicle_attitude_s _vehicle_attitude{};
	vehicle_local_position_s _local_pos{};

	// Target state from GPS (follow_target)
	MapProjection _map_ref{};
	matrix::Vector3f _target_pos{};
	matrix::Vector3f _target_vel{};
	bool _target_pos_valid{false};
	hrt_abstime _last_target_update{0};

	// Visual detection state from seeker camera (target_detection)
	target_detection_s _target_detection{};
	bool _visual_contact{false};
	float _visual_range{0.f};
	hrt_abstime _last_visual_contact{0};

	// Coasting velocity and yaw during visual frame drops
	matrix::Vector3f _last_vel_cmd{};
	float _last_yaw_cmd{0.f};
	bool _last_cmd_valid{false};

	// Predictive intermediate extrapolation (100 Hz guidance with 50/80 FPS camera)
	matrix::Vector3f _vel_cmd_dot{};
	float _yaw_cmd_dot{0.f};
	hrt_abstime _last_fresh_visual_time{0};
	bool _has_fresh_visual{false};

	// Continuous forward speed and acceleration state (unified across APPROACH & CLOSE_PURSUIT)
	float _fwd_speed_cmd{45.f};
	float _fwd_accel_cmd{0.f};
	bool _speed_initialized{false};

	// Optical looming state (IBVS PD)
	float _last_w{0.50f};
	float _last_cx{0.5f};
	float _last_cy{0.5f};
	float _filt_d_w_dt{0.f};
	hrt_abstime _last_visual_time{0};

	// Approach guidance closing rate state
	float _last_approach_dist{-1.f};
	float _filt_closing_speed{0.f};
	hrt_abstime _last_approach_time{0};

	// Line-of-sight unit vector in inertial NED frame
	matrix::Vector3f _last_los_ned{};
	bool _los_ned_valid{false};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::INT_ENABLE>) _param_int_enable,
		(ParamFloat<px4::params::INT_TGT_SIZE>) _param_int_tgt_size,
		(ParamFloat<px4::params::INT_MIN_CLOSING>) _param_int_min_closing,
		(ParamFloat<px4::params::INT_KP_OPT>) _param_int_kp_opt,
		(ParamFloat<px4::params::INT_KD_OPT>) _param_int_kd_opt,
		(ParamFloat<px4::params::MPC_ACC_HOR>) _param_mpc_acc_hor,
		(ParamFloat<px4::params::INT_KP_LAT>) _param_int_kp_lat,
		(ParamFloat<px4::params::INT_KP_Z>) _param_int_kp_z,
		(ParamFloat<px4::params::INT_KP_Z_APP>) _param_int_kp_z_app,
		(ParamFloat<px4::params::INT_YAW_RATE>) _param_int_yaw_rate,
		(ParamFloat<px4::params::INT_K_CORNER>) _param_int_k_corner,
		(ParamFloat<px4::params::INT_LOST_TIMEOUT>) _param_int_lost_timeout
	)
};
