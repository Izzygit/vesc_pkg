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
//#include "vesc_c_if.h"

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
	rt->rt.pitch_angle = rad2deg(VESC_IF->imu_get_pitch());
	rt->yaw_angle = rad2deg(VESC_IF->ahrs_get_yaw(&rt->m_att_ref));
	VESC_IF->imu_get_gyro(rt->gyro);
	VESC_IF->imu_get_accel(rt->accel); //Used for drop detection
	apply_angle_drop(&d->drop, &d->rt); //corrects accel z with angles
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
