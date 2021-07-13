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
 * @file lqr_longitudinal_controller.h
 * Definition of class for longitudinal controller
 *
 * @author Mahmoud Elewa <mhdgamal98@gmail.com>
 *
 * Acknowledgements:
 *
 *   The control design is based on a design
 *   by Youssef Madany and Mahmoud ElQoulaly, 2021,
 *   which can be found as a part of HAPS control
 *   graduation project 2021
 *   Faculty of Engineering, Cairo University.
 */

#ifndef LQR_LONGITUDINAL_CONTROLLER_H
#define LQR_LONGITUDINAL_CONTROLLER_H

#include "ecl_controller.h" // needed for ECL_ControlData struct
#include <mathlib/mathlib.h>
#include <px4_log.h>

class LQR_LONGITUDINAL_CONTROLLER
{
public:
	LQR_LONGITUDINAL_CONTROLLER();
	~LQR_LONGITUDINAL_CONTROLLER() = default;

	float control_attitude_elevator_LQR(const float dt, const ECL_ControlData &ctl_data);

	/* Setters */
	void set_states_0(float u_0, float w_0, float q_0, float th_0)
	{
		_u0  = u_0;
		_w0  = w_0;
		_q0  = q_0;
		_th0 = th_0;
	}

	void set_lqr_gains(float gain_u, float gain_w, float gain_q, float gain_th,  float gain_intg_th)
	{
		_k_ele_u       = gain_u;
		_k_ele_w       = gain_w;
		_k_ele_q       = gain_q;
		_k_ele_th      = gain_th;
		_k_ele_intg_th = gain_intg_th;
	}

	void set_integrator_max(float max){ _integrator_max = max; }

	/* Getters */
	float get_pitch_integrator(){	return _pitch_error_integrator;	}

	void reset_pitch_integrator(){ _pitch_error_integrator = 0.0f; }


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

	float _integrator_max;
	float _last_elevator_output;
	float _pitch_error_integrator;
	float _pitch_error;

};

#endif // LQR_LONGITUDINAL_CONTROLLER_H
