/****************************************************************************
 *
 *   Copyright (c) 2013-2020 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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

/**
 * @file ecl_pitch_controller.h
 * Definition of a simple orthogonal pitch PID controller.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 * Acknowledgements:
 *
 *   The control design is based on a design
 *   by Paul Riseborough and Andrew Tridgell, 2013,
 *   which in turn is based on initial work of
 *   Jonathan Challinger, 2012.
 */

#ifndef ECL_PITCH_CONTROLLER_H
#define ECL_PITCH_CONTROLLER_H

#include <mathlib/mathlib.h>

#include "ecl_controller.h"

class ECL_PitchController :
	public ECL_Controller
{
public:
	ECL_PitchController() = default;
	~ECL_PitchController() = default;

	float control_attitude(const float dt, const ECL_ControlData &ctl_data) override;
	float control_euler_rate(const float dt, const ECL_ControlData &ctl_data) override;
	float control_bodyrate(const float dt, const ECL_ControlData &ctl_data) override;

	float control_attitude_elevator_LQR(const float dt, const ECL_ControlData &ctl_data);

	/* Additional Setters */
	void set_max_rate_pos(float max_rate_pos)
	{
		_max_rate = max_rate_pos;
	}

	void set_max_rate_neg(float max_rate_neg)
	{
		_max_rate_neg = max_rate_neg;
	}

	void set_bodyrate_setpoint(float rate)
	{
		_bodyrate_setpoint = math::constrain(rate, -_max_rate_neg, _max_rate);
	}

	void set_states_0(float u_0, float w_0, float q_0, float th_0)
	{
		_u0  = u_0;
		_w0  = w_0;
		_q0  = q_0;
		_th0 = th_0;
	}

	void set_gains(float gain_u, float gain_w, float gain_q, float gain_th,  float gain_intg_th)
	{
		_k_ele_u       = gain_u;
		_k_ele_w       = gain_w;
		_k_ele_q       = gain_q;
		_k_ele_th      = gain_th;
		_k_ele_intg_th = gain_intg_th;
	}
private:
	float _u0;
	float _w0;
	float _q0;
	float _th0;

	float _k_ele_u;
	float _k_ele_w;
	float _k_ele_q;
	float _k_ele_th;
	float _k_ele_intg_th;

	float _pitch_error;

protected:
	float _max_rate_neg{0.0f};
};

#endif // ECL_PITCH_CONTROLLER_H
