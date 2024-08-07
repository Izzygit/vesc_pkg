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
			deactivate_traction(traction, state, rt, traction_dbg, 6);
		} else if (fabsf(rt->proportional) > config->wheelslip_max_angle) {
			deactivate_traction(traction, state, rt, traction_dbg, 4);
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
				if (fabsf(m->accel_avg) < traction->slowed_accel) {	 	
				// First we identify that the wheel has deccelerated due to traciton control
					traction->highaccelon2 = false;	
				} else if (rt->current_time - traction->timeron > config->pitch_filter/100) {	// Time out at 800ms if wheel does not deccelerate
					deactivate_traction(traction, state, rt, traction_dbg, 5);
				}
			} else if (fabsf(m->accel_avg) > traction->end_accel) {
			// Next we check to see if accel magnitude increases from outside forces 
				deactivate_traction(traction, state, rt, traction_dbg, 2);
			}

			//If we wheelslipped backwards we just need to know the wheel is travelling forwards again
			if (traction->reverse_wheelslip && 
			    m->erpm_sign == m->erpm_sign_soft) {
				deactivate_traction(traction, state, rt, traction_dbg, 3);
			}
		}
	} else { //Start conditions and traciton control activation
		if (traction->end_accel_hold) { //Do not allow start conditions if we are in hold
			traction->end_accel_hold = fabsf(m->accel_avg) > traction->end_accel; //deactivate hold when below the threshold acceleration
		} else { //Start conditions
			//Check motor erpm and acceleration to determine the correct detection condition to use if any
			if (m->erpm_sign == sign(m->erpm_history[m->last_erpm_idx])) { 							//Check sign of the motor at the start of acceleration
				if (fabsf(m->erpm_filtered) > fabsf(m->erpm_history[m->last_erpm_idx])) { 						//If signs the same check for magnitude increase
					start_condition1 = (sign(m->current) * m->accel_avg > traction->start_accel * erpmfactor) &&	// The wheel has broken free indicated by abnormally high acceleration in the direction of motor current
					    (!state->braking_pos);									// Do not apply for braking 
				} // else if (...TODO Put working braking condition here
			} else if (sign(m->erpm_sign_soft) != sign(m->accel_avg)) {				// If the motor is back spinning engage but don't allow wheelslip on landing
				start_condition2 = (sign(m->current) * m->accel_avg > traction->start_accel * erpmfactor) &&	// The wheel has broken free indicated by abnormally high acceleration in the direction of motor current
			   	    (!state->braking_pos);									// Do not apply for braking 
			}
		}
		
		// Initiate traction control
		if ((start_condition1 || start_condition2) && 			// Conditions false by default
		   (rt->current_time - traction->timeroff > 0.02)) {		// Did not recently wheel slip.
			state->wheelslip = true;
			traction->accelstartval = m->accel_avg;
			traction->highaccelon1 = true;
			traction->highaccelon2 = true;
			traction->timeron = rt->current_time;
			if (start_condition2)
				traction->reverse_wheelslip = true;

			//Debug Section
			if (rt->current_time - traction_dbg->aggregate_timer > 5) { // Aggregate the number of drop activations in 5 seconds
				traction_dbg->aggregate_timer = rt->current_time;
				traction_dbg->debug5 = 0;
				traction_dbg->debug2 = erpmfactor;		//only record the first traction loss for some debug variables
				traction_dbg->debug6 = m->accel_avg / traction_dbg->freq_factor; 
				traction_dbg->debug9 = m->erpm_filtered;
				traction_dbg->debug3 = m->erpm_history[m->last_erpm_idx];
				traction_dbg->debug4 = 0;
				traction_dbg->debug8 = 0;
			}

			traction_dbg->debug5 += 1; // count number of traction losses
		}
	}
}

void reset_traction(TractionData *traction, State *state, BrakingData *braking) {
	state->wheelslip = false;
	traction->reverse_wheelslip = false;
	traction->end_accel_hold = false;
	braking->active = false; 
	braking->last_active = false; 
	braking->brake_delay = 0;
	braking->count = 0; 
}

void deactivate_traction(TractionData *traction, State *state, RuntimeData *rt, TractionDebug *traction_dbg, float exit) {
	state->wheelslip = false;
	traction->timeroff = rt->current_time;
	traction->reverse_wheelslip = false;
	traction->end_accel_hold = true; //activate high accel hold to prevent traction control
	if (traction_dbg->debug5 == 1) //only save the first activation duration
		traction_dbg->debug8 = traction->timeroff - traction->timeron;
	if (traction_dbg->debug4 > 10000) 
		traction_dbg->debug4 = traction_dbg->debug4 % 10000;
	traction_dbg->debug4 = traction_dbg->debug4 * 10 + exit; //aggregate the last traction deactivations
}

void configure_traction(TractionData *traction, tnt_config *config, TractionDebug *traction_dbg, BrakingDebug *braking_dbg){
	traction->start_accel = 1000.0 * config->wheelslip_accelstart / config->hertz; //convert from erpm/ms to erpm/cycle
	traction->slowed_accel = 1000.0 * config->wheelslip_accelslowed / config->hertz;
	traction->end_accel = 1000.0 * config->wheelslip_accelend / config->hertz;
	traction_dbg->freq_factor = 1000.0 / config->hertz;
	braking_dbg->freq_factor = traction_dbg->freq_factor;
}

void check_traction_braking(MotorData *m, BrakingData *braking, State *state, RuntimeData *rt, tnt_config *config, float inputtilt_interpolated, BrakingDebug *braking_dbg){
	bool check_last = braking->last_active ||  rt->current_time - braking->brake_delay > config->tc_braking_delay; //we were just traction braking or we are beyond the brake delay

	//Check that conditions for traciton braking are satified and add to counter
	if (-inputtilt_interpolated * m->erpm_sign >= config->tc_braking_angle && //Minimum nose down angle from remote, can be 0
	    state->braking_pos &&						// braking position active
	    m->duty_filtered > config->tc_braking_duty_limit / 100.0) {		// above the minimum duty
		braking->count +=1;
	} else { braking->count = 0; }

	if (braking->count > config->tc_braking_count && //If the counter exceeds the minimum
	    check_last) {				// and the braking delay are satified, allow traction braking
		braking->active = true;
		braking->brake_delay = rt->current_time; //reset delay counter for when we exit traciton braking
		
		//Debug Section
		if (rt->current_time - braking_dbg->aggregate_timer > 5) { // Reset these values after we have not braked for a few seconds
			braking_dbg->debug5 = 0;
			braking_dbg->debug8 = 0;
			braking_dbg->debug6 = 0;
			braking_dbg->debug4 = 0;
			braking_dbg->debug1 = 0;
			braking_dbg->debug3 = 0;
			braking_dbg->debug9 = 0;
		}
		braking_dbg->aggregate_timer = rt->current_time;
		if (!braking->last_active) // Just entered traction braking, reset
			braking->timeron = rt->current_time;
		braking_dbg->debug2 = m->duty_filtered;
		braking_dbg->debug6 = max(braking_dbg->debug6, fabsf(m->accel_avg / braking_dbg->freq_factor));
		braking_dbg->debug9 = max(fabsf(braking_dbg->debug9), m->abs_erpm) * m->erpm_sign;
		if (braking_dbg->debug3 == 0)
			braking_dbg->debug3 = m->erpm;
		braking_dbg->debug3 = min(fabsf(braking_dbg->debug3), m->abs_erpm) * m->erpm_sign;	
		braking_dbg->debug8 = rt->current_time - braking->timeron + braking_dbg->debug1; //running on time tracker
	} else { 
		braking->active = false; 
		
		//Debug Section
		if (braking->last_active) {
			braking->timeroff = rt->current_time;
			braking_dbg->debug1 += braking->timeroff - braking->timeron; //sum all activation times
			braking_dbg->debug8 = braking_dbg->debug1; //deactivated on time tracker
			braking_dbg->debug5 += 1; //count deactivations
		
			if (braking_dbg->debug4 > 10000)  //Save 5 of the most recent deactivation reasons
				braking_dbg->debug4 = braking_dbg->debug4 % 10000;
			
			if (-inputtilt_interpolated * m->erpm_sign < config->tc_braking_angle) {
				braking_dbg->debug4 = braking_dbg->debug4 * 10 + 1;
			} else if (!state->braking_pos) {
				braking_dbg->debug4 = braking_dbg->debug4 * 10 + 2;
			} else if (m->duty_filtered < config->tc_braking_duty_limit / 100.0) {
				braking_dbg->debug4 = braking_dbg->debug4 * 10 + 3;
			}
		}
	}
	braking->last_active = braking->active;
}
