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
#include "runtime.h"
#include "state_tnt.h"
#include "vesc_c_if.h"
#include "motor_data_tnt.h"

typedef struct {
	float freq[3];
	float voltage;
	float duration;
	int priority;
	int times;
	bool tone_in_progress;
	float timer;
	bool pause;
	float pause_timer;
	int beep_reason;
	bool midvolt_activated;
	bool lowvolt_activated;
	bool motortemp_activated;
	bool fettemp_activated;
	float idle_voltage;
} ToneData;

typedef struct {
	float freq[3];
	float voltage;
	float duration;
	int priority;
	int times;
	float delay;
} ToneConfig;

typedef struct {
	ToneConfig continuous1;
	ToneConfig continuousfootpad;
	ToneConfig fastdouble1;
	ToneConfig fastdouble2;
	ToneConfig slowdouble1;
	ToneConfig slowdouble2;
	ToneConfig fasttriple1;
	ToneConfig fasttriple2;
	ToneConfig slowtriple1;
	ToneConfig slowtriple2;
	ToneConfig fasttripleup;
	ToneConfig fasttripleupduty;
	ToneConfig fasttripledown;
	ToneConfig slowtripleup;
	ToneConfig slowtripledown;
	ToneConfig dutytone;
	ToneConfig currenttone;
} ToneConfigs;

void tone_update(ToneData *tone, RuntimeData *rt, State *state);
void play_tone(ToneData *tone, ToneConfig *toneconfig, RuntimeData *rt, int beep_reason);
void end_tone(ToneData *tone);
void tone_reset(ToneData *tone);
void tone_configure(ToneConfig *toneconfig, float freq1, float freq2, float freq3, float voltage, float duration, int times, float delay, int priority);
void tone_configure_all(ToneConfigs *toneconfig, tnt_config *config);
void idle_tone(ToneData *tone, ToneConfig *toneconfig, RuntimeData *rt);
void temp_recovery_tone(ToneData *tone, ToneConfig *toneconfig, RuntimeData *rt, MotorData *motor);
