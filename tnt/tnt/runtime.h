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

#pragma once
#include "biquad.h"
#include "kalman.h"
#include "conf/datatypes.h"
#include "vesc_c_if.h"

typedef struct { //Run time values used in various features
	float proportional;
	float pid_value;
	float pitch_angle;
	float roll_angle;
	float yaw_angle;
	float current_time;
	float setpoint;
	float last_accel_z;
	float accel[3];
	float abs_roll_angle;
 	float true_pitch_angle;
	float gyro[3];
	float pitch_smooth; // Low Pass Filter
	Biquad pitch_biquad; // Low Pass Filter
	KalmanFilter pitch_kalman; // Kalman Filter
	float pitch_smooth_kalman; // Kalman Filter
	float diff_time, last_time;
	ATTITUDE_INFO m_att_ref; // Feature: True Pitch / Yaw
} RuntimeData;

void runtime_data_update(RuntimeData *rt);
void apply_pitch_filters(RuntimeData *rt, tnt_config *config);
