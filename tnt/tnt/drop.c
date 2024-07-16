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

#include "drop.h"
#include <math.h>
#include "utils_tnt.h"

void check_drop(DropData *drop, MotorData *m, RuntimeData *rt, State *state, DropDebug *drop_dbg){
	//Conditions to engage drop
	if ((drop->accel_z < drop->z_limit) && 						// Compare accel z to drop limit with reduction for pitch and roll.
	    (rt->last_accel_z >= drop->accel_z)) {  					// check that we are constantly dropping
		drop->count += 1;
	} else { drop-> count = 0; }

	if ((drop->count > 10) &&	
	    (state->sat != SAT_CENTERING) && 						// Not during startup
	    (rt->current_time - drop->timeroff > 0.02)) {				// Don't re-enter drop state for duration 	
		
		//Debug Section
		if (!drop->active) { 						// Set the on timer only once per drop
			drop->timeron = rt->current_time; 
			drop_dbg->setpoint = rt->setpoint;
			if (rt->current_time - drop_dbg->aggregate_timer > 5) { // Aggregate the number of drop activations in 5 seconds
				drop_dbg->aggregate_timer = rt->current_time; //reset
				drop_dbg->debug5 = 0; //reset
				drop_dbg->debug4 = drop->accel_z; //reset
				drop_dbg->debug7 = 0; //reset
			}
			drop_dbg->debug5 += 1;
		}
		
		//Activate drop mode
		drop->active = true;
	}
	
	// Conditions to end drop
	if (drop->active == true) {				
		drop_dbg->debug4 = min(drop_dbg->debug4, drop->accel_z); 	//record the lowest accel
		if (fabsf(m->accel_filtered) > drop->motor_limit) { 	//Fastest reaction is hall sensor
			drop_deactivate(drop, drop_dbg, rt);
			drop_dbg->debug3 = m->accel_filtered;
		} else if (rt->last_accel_z <= drop->accel_z) {		// for fastest landing reaction with accelerometer check that we are still dropping
			drop_deactivate(drop, drop_dbg, rt);
			drop_dbg->debug3 = drop->accel_z;
		}
	}		
}

void configure_drop(DropData *drop, const tnt_config *config){
	drop->z_limit = config->drop_z_accel / 100.0;	// Value of accel z to initiate drop. A drop of about 6" / .1s produces about 0.9 accel y (normally 1)
	drop->motor_limit = 1000.0 * 2.0 / config->hertz; //ends drop via motor acceleration config->drop_motor_accel
}

void reset_drop(DropData *drop){
	drop->active = false;
}

void drop_deactivate(DropData *drop, DropDebug *drop_dbg, RuntimeData *rt){
	drop->active = false;
	drop->timeroff = rt->current_time;
	drop_dbg->debug7 += (drop->timeroff - drop->timeron);
	drop_dbg->debug6 = drop_dbg->setpoint - rt->pitch_angle;
}

void apply_angle_drop(DropData *drop, RuntimeData *rt){
	rt->last_accel_z = drop->accel_z;
	float angle_correction = max(.1, min(10, 1 / (cosf(deg2rad(rt->roll_angle)) * cosf(deg2rad(rt->pitch_angle)))));		// Accel z is naturally reduced by the pitch and roll angles, so use geometry to compensate
	if (drop->applied_correction < angle_correction) {								// Accel z acts slower than pitch and roll so we need to delay accel z reduction as necessary
		drop->applied_correction = angle_correction ;							// Roll or pitch are increasing. Do not delay
	} else {
		drop->applied_correction = drop->applied_correction * (1 - .001) + angle_correction * .001;		// Roll or pitch decreasing. Delay to allow accelerometer to keep pace	
	}
	drop->accel_z = rt->accel[2] * drop->applied_correction;
	/* TODO Another method that incorporates all accelerations but needs work on delay.
	float rad_pitch = deg2rad(rt->pitch_angle);
	float rad_roll =deg2rad(rt->roll_angle);
	float angle_factor = cosf(drop->pitch_delay) * cosf(drop->roll_delay);
	if (drop->last_angle_factor > angle_factor) {					// Accel z acts slower than pitch and roll so we need to delay accel z reduction as necessary
		drop->pitch_delay = rad_pitch;						// Roll or pitch are increasing. Do not delay
		drop->roll_delay = rad_roll;	
	} else {
		drop->pitch_delay = drop->pitch_delay* (1 - .001) + rad_pitch * .001;		// Roll or pitch decreasing. Delay to allow accelerometer to keep pace	
		drop->roll_delay = drop->roll_delay* (1 - .001) + rad_roll * .001;	
	}
	drop->last_angle_factor = angle_factor;

	drop->accel_z =  rt->accel[2] * angle_factor +  
		fabsf(rt->accel[1] * sinf(drop->roll_delay)) +  
		fabsf(rt->accel[0] * sinf(drop->pitch_delay)) ; */
}
