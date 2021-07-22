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

#include "lqr_longitudinal_controller.h"
#include <float.h>
#include <lib/geo/geo.h>
#include <mathlib/mathlib.h>

LQR_LONGITUDINAL_CONTROLLER::LQR_LONGITUDINAL_CONTROLLER() :
	_u0(0.0f),
	_w0(0.0f),
	_q0(0.0f),
	_th0(0.0f),
	_k_ele_u(0.0f),
	_k_ele_w(0.0f),
	_k_ele_q(0.0f),
	_k_ele_th(0.0f),
	_k_ele_intg_th(0.0f),
	_integrator_max(0.0f),
	_last_elevator_output(0.0f),
	_pitch_error_integrator(0.0f),
	_pitch_error(0.0f)
{
}

float LQR_LONGITUDINAL_CONTROLLER::control_attitude_elevator_LQR(const float dt, const ECL_ControlData &ctl_data)
{
	/* Do not calculate control signal with bad inputs */
	if (!(PX4_ISFINITE(ctl_data.u) &&
	      PX4_ISFINITE(ctl_data.w) &&
	      PX4_ISFINITE(ctl_data.pitch) &&
	      PX4_ISFINITE(ctl_data.body_y_rate) &&
	      PX4_ISFINITE(ctl_data.pitch_setpoint) &&
	      PX4_ISFINITE(ctl_data.airspeed_min) &&
	      PX4_ISFINITE(ctl_data.airspeed_max) &&
	      PX4_ISFINITE(ctl_data.scaler))) {

		return math::constrain(_last_elevator_output, -1.0f, 1.0f);
	}

	/* Calculate pitch error */
	_pitch_error = ctl_data.pitch_setpoint - ctl_data.pitch;

	/* Calculate delta_states */
	float delta_u  = ctl_data.u-_u0;
	float delta_w  = ctl_data.w-_w0;
	float delta_q  = ctl_data.body_y_rate-_q0;
	float delta_th = ctl_data.pitch-_th0;

	if (!ctl_data.lock_integrator) {

		/* Integral term */
		float id = _pitch_error * dt;

		/*
		 * anti-windup: do not allow integrator to increase if actuator is at limit
		 */
		if (_last_elevator_output < -1.0f) {
			/* only allow motion to center: increase value */
			id = math::max(id, 0.0f);

		} else if (_last_elevator_output > 1.0f) {
			/* only allow motion to center: decrease value */
			id = math::min(id, 0.0f);
		}

		/* add and constrain */
		//_pitch_error_integrator = math::constrain(_pitch_error_integrator + id , -_integrator_max, _integrator_max);
		_pitch_error_integrator = _pitch_error_integrator + id;
	}

	/* Apply LQR controller and store non-limited output */
	_last_elevator_output = _k_ele_u * delta_u + _k_ele_w * delta_w + _k_ele_q * delta_q + _k_ele_th * delta_th + _k_ele_intg_th * _pitch_error_integrator;
	/* u = kx not u ~= -kx due to different sign convention for servo */
	_last_elevator_output = _last_elevator_output * 1.0f;
	_last_elevator_output = _last_elevator_output * 2.0f;
	return math::constrain(_last_elevator_output, -1.0f, 1.0f);
}
