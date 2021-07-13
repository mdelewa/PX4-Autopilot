/****************************************************************************
 *
 *   Copyright (c) 2021 HAPS Control Graduation Project. All rights reserved.
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
 * 3. Neither the name HAPS Control GP nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
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

/**
 * @file lqr_longitudinal_controller.cpp
 * Implementation of a longitudinal pitch LQR controller.
 *
 * Authors and acknowledgements in header.
 */

#include "lqr_lateral_controller.h"
#include <float.h>
#include <lib/ecl/geo/geo.h>
#include <mathlib/mathlib.h>

LQR_LATERAL_CONTROLLER::LQR_LATERAL_CONTROLLER() :
	_k_ail_v(0.0f),
	_k_ail_p(0.0f),
	_k_ail_r(0.0f),
	_k_ail_ph(0.0f),
	_k_ail_intg_ph(0.0f),
	_k_rud_v(0.0f),
	_k_rud_p(0.0f),
	_k_rud_r(0.0f),
	_k_rud_ph(0.0f),
	_k_rud_intg_ph(0.0f),
	_integrator_max(0.0f),
	_last_aileron_output(0.0f),
	_last_rudder_output(0.0f),
	_roll_error_integrator(0.0f),
	_roll_error(0.0f)
{
}

Vector2f LQR_LATERAL_CONTROLLER::control_attitude_aileron_rudder_LQR(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.v) &&
	      PX4_ISFINITE(ctl_data.body_x_rate) &&
	      PX4_ISFINITE(ctl_data.body_z_rate) &&
	      PX4_ISFINITE(ctl_data.roll) &&
	      PX4_ISFINITE(ctl_data.roll_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {

		      float last_ail_constrained = math::constrain(_last_aileron_output, -1.0f, 1.0f);
		      float last_rud_constrained = math::constrain(_last_rudder_output, -1.0f, 1.0f);

		return Vector2f(last_ail_constrained, last_rud_constrained);
	}

	/* Calculate roll error */
	_roll_error = ctl_data.roll_setpoint - ctl_data.roll;

	/* Calculate delta_states */
	float delta_v  = ctl_data.v;
	float delta_p  = ctl_data.body_x_rate;
	float delta_r  = ctl_data.body_z_rate;
	float delta_ph = ctl_data.roll;

	if (!ctl_data.lock_integrator) {

		/* Integral term */
		float id = _roll_error * dt;

		/*
		 * anti-windup: do not allow integrator to increase if actuator is at limit
		 */
		if (_last_aileron_output < -1.0f || _last_rudder_output < -1.0f) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);

		} else if (_last_aileron_output > 1.0f || _last_rudder_output > 1.0f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		/* add and constrain */
		_roll_error_integrator = math::constrain(_roll_error_integrator + id , -_integrator_max, _integrator_max);
	}

	/* Apply LQR controller and store non-limited output */

	_last_aileron_output = _k_ail_v * delta_v * 0.0f + _k_ail_p * delta_p * 1.0f + _k_ail_r * delta_r + _k_ail_ph * delta_ph + _k_ail_intg_ph * _roll_error_integrator;
	_last_aileron_output = _last_aileron_output*-1;
	_last_aileron_output = _last_aileron_output*2;

	_last_rudder_output = _k_rud_v * delta_v * 0.0f  + _k_rud_p * delta_p * 1.0f + _k_rud_r * delta_r + _k_rud_ph * delta_ph + _k_rud_intg_ph * _roll_error_integrator;
	_last_rudder_output = _last_rudder_output*1;
	_last_rudder_output = _last_rudder_output*2;

	float last_ail_constrained = math::constrain(_last_aileron_output, -1.0f, 1.0f);
	float last_rud_constrained = math::constrain(_last_rudder_output, -1.0f, 1.0f);

	return Vector2f(last_ail_constrained, last_rud_constrained);
}
