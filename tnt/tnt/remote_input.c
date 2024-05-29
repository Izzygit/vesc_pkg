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

#include "remote_input.h"
#include "utils_tnt.h"
#include <math.h>

void update_remote(tnt_config *config, RemoteData *r) {
	// UART/PPM Remote Throttle 
	bool remote_connected = false;
	float servo_val = 0;
	switch (config->inputtilt_remote_type) {
	case (INPUTTILT_PPM):
		servo_val = VESC_IF->get_ppm();
		remote_connected = VESC_IF->get_ppm_age() < 1;
		break;
	case (INPUTTILT_UART): ; // Don't delete ";", required to avoid compiler error with first line variable init
		remote_state remote = VESC_IF->get_remote_state();
		servo_val = remote.js_y;
		remote_connected = remote.age_s < 1;
		break;
	case (INPUTTILT_NONE):
		break;
	}

	if (!remote_connected) {
		servo_val = 0;
	} else {
		// Apply Deadband
		float deadband = config->inputtilt_deadband;
		if (fabsf(servo_val) < deadband) {
			servo_val = 0.0;
		} else {
			servo_val = sign(servo_val) * (fabsf(servo_val) - deadband) / (1 - deadband);
		}
		// Invert Throttle
		servo_val *= (config->inputtilt_invert_throttle ? -1.0 : 1.0);
	}
	r->throttle_val = servo_val;
}
