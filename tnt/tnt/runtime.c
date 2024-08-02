// Copyright 2024 Lukas Hrazky
//
// This file is part of the Refloat VESC package.
//
// Refloat VESC package is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// Refloat VESC package is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.

#include "runtime.h"
#include "vesc_c_if.h"
#include <math.h>
#include "utils_tnt.h"
#include "biquad.h"
#include "kalman.h"

void runtime_data_update(RuntimeData *rt) {
	// Update times
	rt->current_time = VESC_IF->system_time();
	if (rt->last_time == 0) {
		rt->last_time = rt->current_time;
	}
	rt->diff_time = rt->current_time - rt->last_time;
	rt->last_time = rt->current_time;
	
	// Get the IMU Values
	rt->roll_angle = rad2deg(VESC_IF->imu_get_roll());
	rt->abs_roll_angle = fabsf(rt->roll_angle);
	rt->true_pitch_angle = rad2deg(VESC_IF->ahrs_get_pitch(&rt->m_att_ref)); // True pitch is derived from the secondary IMU filter running with kp=0.2
	rt->pitch_angle = rad2deg(VESC_IF->imu_get_pitch());
	rt->yaw_angle = rad2deg(VESC_IF->ahrs_get_yaw(&rt->m_att_ref));
	VESC_IF->imu_get_gyro(rt->gyro);
	VESC_IF->imu_get_accel(rt->accel); //Used for drop detection
}

void apply_pitch_filters(RuntimeData *rt, tnt_config *config){
	//Apply low pass and Kalman filters to pitch
	if (config->pitch_filter > 0) {
		rt->pitch_smooth = biquad_process(&rt->pitch_biquad, rt->pitch_angle);
	} else {rt->pitch_smooth = rt->pitch_angle;}
	if (config->kalman_factor1 > 0) {
		 apply_kalman(rt->pitch_smooth, rt->gyro[1], &rt->pitch_smooth_kalman, rt->diff_time, &rt->pitch_kalman);
	} else {rt->pitch_smooth_kalman = rt->pitch_smooth;}
}

void calc_yaw_change(YawData *yaw, float yaw_angle, YawDebugData *yaw_dbg){ 
	float new_change = yaw_angle - yaw->last_angle;
	if ((new_change == 0) || // Exact 0's only happen when the IMU is not updating between loops
	    (fabsf(new_change) > 100)) { // yaw flips signs at 180, ignore those changes
		new_change = yaw->last_change;
	}
	yaw->last_change = new_change;
	yaw->last_angle = yaw_angle;
	yaw->change = yaw->change * 0.8 + 0.2 * (new_change);
	yaw->abs_change = fabsf(yaw->change);
	yaw_dbg->debug1 = yaw->change;
}

void yaw_reset(YawData *yaw, YawDebugData *yaw_dbg){ 
	yaw->last_angle = 0;
	yaw->last_change = 0;
	yaw->abs_change = 0;
	yaw_dbg->debug2 = 0;
}

void reset_runtime(RuntimeData *rt) {
	//Low pass pitch filter
	rt->pitch_smooth = rt->pitch_angle;
	biquad_reset(&rt->pitch_biquad);
	
	//Kalman filter
	reset_kalman(&rt->pitch_kalman);
	rt->pitch_smooth_kalman = rt->pitch_angle;
}
