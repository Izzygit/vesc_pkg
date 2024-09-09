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

#include "setpoint.h"
#include "utils_tnt.h"

void setpoint_configure(SetpointData *s, tnt_config *config) {
	//Setpoint Adjustment
	s->startup_step_size = 1.0 * config->startup_speed / config->hertz;
	s->tiltback_duty_step_size = 1.0 * config->tiltback_duty_speed / config->hertz;
	s->tiltback_hv_step_size = 1.0 * config->tiltback_hv_speed / config->hertz;
	s->tiltback_lv_step_size = 1.0 * config->tiltback_lv_speed / config->hertz;
	s->tiltback_return_step_size = 1.0 * config->tiltback_return_speed / config->hertz;
	s->tiltback_ht_step_size = 1.0 * config->tiltback_ht_speed / config->hertz;
	s->noseangling_step_size = 1.0 * config->noseangling_speed / config->hertz;
	s->tiltback_duty = 1.0 * config->tiltback_duty / 100.0;
	s->surge_tiltback_step_size = 1.0 * config->tiltback_surge_speed / config->hertz;

	// Feature: Dirty Landings
	s->startup_pitch_trickmargin = config->startup_dirtylandings_enabled ? 10 : 0;
}

void setpoint_reset(SetpointData *s, tnt_config *config, RuntimeData *rt) {
	// Set values for startup
	s->setpoint_target_interpolated = rt->pitch_angle;
	s->setpoint_target = 0;
	s->startup_pitch_tolerance = config->startup_pitch_tolerance;
	s->setpoint = rt->pitch_angle;
}

float get_setpoint_adjustment_step_size(SetpointData *s, State *state) {
	switch (state->sat) {
	case (SAT_NONE):
		return s->tiltback_return_step_size;
	case (SAT_CENTERING):
		return s->startup_step_size;
	case (SAT_PB_DUTY):
		return s->tiltback_duty_step_size;
	case (SAT_PB_HIGH_VOLTAGE):
		return s->tiltback_hv_step_size;
	case (SAT_PB_TEMPERATURE):
		return s->tiltback_ht_step_size;
	case (SAT_PB_LOW_VOLTAGE):
		return s->tiltback_lv_step_size;
	case (SAT_UNSURGE):
		return s->surge_tiltback_step_size;
	case (SAT_SURGE):
		return 25; 				//"as fast as possible", extremely large step size 
	default:
		return 0;
	}
}

void calculate_setpoint_interpolated(SetpointData *s, State *state) {
    if (s->setpoint_target_interpolated != s->setpoint_target) {
        rate_limitf(
            &s->setpoint_target_interpolated,
            s->setpoint_target,
            get_setpoint_adjustment_step_size(s, state) 
	);
    }
}

void apply_noseangling(SetpointData *s, MotorData *motor, tnt_config *config) {
	float noseangling_target = 0;
	if (motor->abs_erpm > config->tiltback_constant_erpm) {
		noseangling_target += config->tiltback_constant * motor->erpm_sign;
	}

	rate_limitf(&s->noseangling_interpolated, noseangling_target, s->noseangling_step_size);

	s->setpoint += s->noseangling_interpolated;
}
