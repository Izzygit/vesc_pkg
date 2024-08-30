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

#include "conf/datatypes.h"
#include "remote_input.h"
#include "runtime.h"
#include "motor_data_tnt.h"
#include "state_tnt.h"
#include "vesc_c_if.h"

typedef struct {
	float angle_kp[7][2];
	int count;
	float kp_rate;
} KpArray;

typedef struct {
	float proportional;
	float pid_value;
	float prop_smooth;
	float abs_prop_smooth;
	float pid_mod;
	float stabl;
	float stability_kp;
	float stability_kprate;
	float stabl_step_size_up, stabl_step_size_down;
	float roll_pid_mod;
	float yaw_pid_mod;
	float softstart_pid_limit;
	float softstart_step_size;
	bool brake_pitch, brake_roll, brake_yaw;
} PidData;

typedef struct {
	float freq[2];
	float voltage;
	float duration;
	int priority;
	int times;
	bool tone_in_progress;
	float timer;
	bool pause;
	float pause_timer;
} ToneData;

typedef struct {
	float freq[2];
	float voltage;
	float duration;
	int priority;
	int times;
} ToneConfig;

typedef struct {
	ToneConfig continuous1;
	ToneConfig continuous2;
	ToneConfig fastdouble1;
	ToneConfig fastdouble2;
	ToneConfig slowdouble1;
	ToneConfig slowdouble2;
	ToneConfig fasttriple1;
	ToneConfig fasttriple2;
	ToneConfig slowtriple1;
	ToneConfig slowtriple2;
	ToneConfig fasttripleup;
	ToneConfig fasttripledown;
	ToneConfig slowtripleup;
	ToneConfig slowtripledown;
	ToneConfig dutytone;
	ToneConfig currenttone;
} ToneConfigs;

void pitch_kp_configure(const tnt_config *config, KpArray *k, int mode);
void roll_kp_configure(const tnt_config *config, KpArray *k, int mode);
void yaw_kp_configure(const tnt_config *config, KpArray *k, int mode);
float angle_kp_select(float angle, const KpArray *k);
void angle_kp_reset(KpArray *k);
float erpm_scale(float lowvalue, float highvalue, float lowscale, float highscale, float abs_erpm); 
void apply_stability(PidData *p, MotorData *m, RemoteData *remote, tnt_config *config);
void check_brake_kp(PidData *p, State *state, tnt_config *config, KpArray *roll_brake_kp, KpArray *yaw_brake_kp);
float roll_erpm_scale(PidData *p, State *state, MotorData *m, KpArray *roll_accel_kp, tnt_config *config);
void reset_pid(PidData *p);
void apply_soft_start(PidData *p, MotorData *m);
void configure_pid(PidData *p, tnt_config *config);
void tone_update(ToneData *tone, RuntimeData *rt, State *state);
void play_tone(ToneData *tone, ToneConfig *toneconfig);
void end_tone(ToneData *tone);
void tone_reset(ToneData *tone);
void tone_configure(ToneConfig *toneconfig, float freq, float voltage, float duration, int times, int priority);
void tone_configure_all(ToneConfigs *toneconfig, tnt_config *config);
