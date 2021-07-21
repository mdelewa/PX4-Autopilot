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
 * @file lqr_lateral_controller.h
 * Definition of class for lateral controller
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

#ifndef LQR_LATERAL_CONTROLLER_H
#define LQR_LATERAL_CONTROLLER_H

#include "ecl_controller.h" // needed for ECL_ControlData struct
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_log.h>

using matrix::Vector2f;

class LQR_LATERAL_CONTROLLER
{
public:
	LQR_LATERAL_CONTROLLER();
	~LQR_LATERAL_CONTROLLER() = default;

	Vector2f control_attitude_aileron_rudder_LQR(const float dt, const ECL_ControlData &ctl_data);

	/* Setters */

	void set_lqr_aileron_gains(float gain_v, float gain_p, float gain_r, float gain_ph,  float gain_intg_ph)
	{
		_k_ail_v       = gain_v;
		_k_ail_p       = gain_p;
		_k_ail_r       = gain_r;
		_k_ail_ph      = gain_ph;
		_k_ail_intg_ph = gain_intg_ph;
	}

	void set_lqr_rudder_gains(float gain_v, float gain_p, float gain_r, float gain_ph,  float gain_intg_ph)
	{
		_k_rud_v       = gain_v;
		_k_rud_p       = gain_p;
		_k_rud_r       = gain_r;
		_k_rud_ph      = gain_ph;
		_k_rud_intg_ph = gain_intg_ph;
	}


	void set_integrator_max(float max){ _integrator_max = max; }

	/* Getters */
	//float get_roll_integrator(){ return _roll_error_integrator; }

	void reset_roll_integrator_ail(){ _roll_error_integrator_ail = 0.0f; }
	void reset_roll_integrator_rud(){ _roll_error_integrator_rud = 0.0f; }


private:

	float _k_ail_v;
	float _k_ail_p;
	float _k_ail_r;
	float _k_ail_ph;
	float _k_ail_intg_ph;

	float _k_rud_v;
	float _k_rud_p;
	float _k_rud_r;
	float _k_rud_ph;
	float _k_rud_intg_ph;

	float _integrator_max;
	float _last_aileron_output;
	float _last_rudder_output;
	float _roll_error_integrator_ail;
	float _roll_error_integrator_rud;
	float _roll_error;

};

#endif // LQR_LATERAL_CONTROLLER_H
