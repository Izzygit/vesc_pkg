// Copyright 2024 Michael Silberstein
//
// This file is part of the VESC package.
//
// This VESC package is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This VESC package is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.

#include "traction.h"
#include <math.h>
#include "utils_tnt.h"

void check_traction(MotorData *m, TractionData *traction, State *state, RuntimeData *rt, tnt_config *config, TractionDebug *traction_dbg){
	float erpmfactor = fmaxf(1, lerp(0, config->wheelslip_scaleerpm, config->wheelslip_scaleaccel, 1, m->abs_erpm));
	bool start_condition1 = false;
	bool start_condition2 = false;

	// Conditions to end traction control
	if (state->wheelslip) {
		if (rt->current_time - traction->timeron > .8) {		// Time out at 500ms
					deactivate_traction(traction, state, rt, traction_dbg, 80);
		} else if (fabsf(rt->proportional) > config->wheelslip_max_angle) {
					deactivate_traction(traction, state, rt, traction_dbg, 10);
		} else {
			//This section determines if the wheel is acted on by outside forces by detecting acceleration direction change
			if (traction->highaccelon1) { 
				if (sign(traction->accelstartval) != sign(m->accel_filtered)) { 
				// First we identify that the wheel has deccelerated due to traciton control, switching the sign
					traction->highaccelon1 = false;				
				} 
			} else if (sign(m->accel_filtered)!= sign(m->last_accel_filtered)) { 
			// Next we check to see if accel direction changes again from outside forces 
					deactivate_traction(traction, state, rt, traction_dbg, 1);
			}
			
			//This section determines if the wheel is acted on by outside forces by detecting acceleration magnitude
			if (traction->highaccelon2) {
				if (sign(traction->accelstartval) * m->accel < traction->slowed_accel) {	 	
				// First we identify that the wheel has deccelerated due to traciton control
					traction->highaccelon2 = false;	
				} else if ((rt->current_time - traction->timeron > 0.5) && 
				    traction->highaccelon1) {					// Time out at 800ms if wheel does not deccelerate
					deactivate_traction(traction, state, rt, traction_dbg, 50);
				}
			} else if (fabsf(m->accel) > traction->end_accel) {
			// Next we check to see if accel magnitude increases from outside forces 
				deactivate_traction(traction, state, rt, traction_dbg, 2);
			}

			//If we wheelslipped backwards we just need to know the wheel is travelling forwards again
			if (traction->reverse_wheelslip && 
			    m->erpm_sign == m->erpm_sign_soft) {
				deactivate_traction(traction, state, rt, traction_dbg, 3);
			}
		}
	} else {
		if (traction->end_accel_hold) { //Do not allow start conditions if we are in hold
			traction->end_accel_hold = fabsf(m->accel) > traction->end_accel; //deactivate hold when below the threshold acceleration
		} else { //Start conditions
			//Check motor erpm and acceleration to determine the correct detection condition to use if any
			if (m->erpm_sign == sign(m->erpm_history[m->last_erpm_idx])) { 							//Check sign of the motor at the start of acceleration
				if (m->abs_erpm > fabsf(m->erpm_history[m->last_erpm_idx])) { 						//If signs the same check for magnitude increase
					start_condition1 = (sign(m->current) * m->accel > traction->start_accel * erpmfactor) &&	// The wheel has broken free indicated by abnormally high acceleration in the direction of motor current
					    (!state->braking_pos);									// Do not apply for braking 
				} // else if (...TODO Put working braking condition here
			} else if (sign(m->erpm_sign_soft) != sign(m->accel)) {				// If the motor is back spinning engage but don't allow wheelslip on landing
				start_condition2 = (sign(m->current) * m->accel > traction->start_accel * erpmfactor) &&	// The wheel has broken free indicated by abnormally high acceleration in the direction of motor current
			   	    (!state->braking_pos);									// Do not apply for braking 
			}
		}
	
		traction_dbg->debug2 = m->erpm_sign_soft;
	
		// Initiate traction control
		if ((start_condition1 || start_condition2) && 			// Conditions false by default
		   (rt->current_time - traction->timeroff > 0.02)) {		// Did not recently wheel slip.
			state->wheelslip = true;
			traction->accelstartval = m->accel;
			traction->highaccelon1 = true;
			traction->highaccelon2 = true;
			traction->timeron = rt->current_time;
			if (start_condition2)
				traction->reverse_wheelslip = true;

			//Debug Section
			if (rt->current_time - traction_dbg->aggregate_timer > 5) { // Aggregate the number of drop activations in 5 seconds
				traction_dbg->aggregate_timer = rt->current_time;
				traction_dbg->debug5 = 0;
			}
			if (traction_dbg->debug5 == 0) {
				//traction_dbg->debug2 = m->erpm_sign_soft;
				traction_dbg->debug6 = m->accel / traction_dbg->freq_factor; 
				traction_dbg->debug9 = m->erpm;
				traction_dbg->debug3 = m->erpm_history[m->last_erpm_idx];
				traction_dbg->debug1 = 0;
				traction_dbg->debug4 = 0;
				traction_dbg->debug8 = 0;
			}
			traction_dbg->debug5 += 1;
		}
	}
}

void reset_traction(TractionData *traction, State *state) {
	state->wheelslip = false;
	traction->reverse_wheelslip = false;
	traction->end_accel_hold = false;
}

void deactivate_traction(TractionData *traction, State *state, RuntimeData *rt, TractionDebug *traction_dbg, float exit) {
	state->wheelslip = false;
	traction->timeroff = rt->current_time;
	traction->reverse_wheelslip = false;
	traction->end_accel_hold = true;
	if (traction_dbg->debug5 == 1) {
		traction_dbg->debug8 = traction->timeroff - traction->timeron;
		if (traction_dbg->debug4 != 0) {
			traction_dbg->debug4 = traction_dbg->debug4 * 100 + exit;
		} else { traction_dbg->debug4 = exit; }
	}
}

void configure_traction(TractionData *traction, tnt_config *config, TractionDebug *traction_dbg){
	traction->start_accel = 1000.0 * config->wheelslip_accelstart / config->hertz; //convert from erpm/ms to erpm/cycle
	traction->slowed_accel = 1000.0 * config->wheelslip_accelslowed / config->hertz;
	traction->end_accel = 1000.0 * config->wheelslip_accelend / config->hertz;
	traction_dbg->freq_factor = 1000.0 / config->hertz;
}

void check_traction_braking(MotorData *m, TractionData *traction, State *state, RuntimeData *rt, tnt_config *config, float inputtilt_interpolated, TractionDebug *traction_dbg){
	if (-inputtilt_interpolated * m->erpm_sign >= config->tc_braking_angle &&
	    state->braking_pos &&
	    m->duty_filtered > config->kalman_factor2) {
		traction->traction_braking = true;
		
		//Debug Section
		traction_dbg->debug2 = 0;
		traction_dbg->debug6 = 666;
		traction_dbg->debug9 = 0;
		traction_dbg->debug3 = 0;
		traction_dbg->debug1 = 0;
		traction_dbg->debug4 = 0;
		if (!traction->traction_braking_last)  // Just entered traction braking, reset
			traction->timeron = rt->current_time;

		traction_dbg->debug8 = rt->current_time - traction->timeron;
	} else { 
		traction->traction_braking = false; 

		//Debug Section
		if (traction->traction_braking_last) {
			traction->timeroff = rt->current_time;
			traction_dbg->debug8 = traction->timeroff - traction->timeron;

			if (rt->current_time - traction_dbg->aggregate_timer > 10) { // Aggregate the number of drop activations in 10 seconds
				traction_dbg->aggregate_timer = rt->current_time;
				traction_dbg->debug5 = 0;
			}
			traction_dbg->debug5 += 1;
		}
	}
	traction->traction_braking_last = traction->traction_braking;
}
