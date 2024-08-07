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
#include "motor_data_tnt.h"
#include "state_tnt.h"
#include "runtime.h"

typedef struct {
        float accel_z;
	bool active;				//Drop is occurring
        float timeron;				//timer for debug info
        float timeroff;				//timer for debug info
        float applied_correction;		//Geometry compesation for the angle of the board
        float z_limit;				//Required acceleration to engage drop
        float motor_limit;			//Required motor acceleration to end drop
	int count;
	int count_limit;
} DropData;

typedef struct {
	float debug1;		//last high accel prevention time
	float debug2;		//
	int debug3;		//end condition
	float debug4;		//min accel z
	float debug5;		// number of drops in 5 seconds
	float debug6;		// end porp
	float debug7;		// drop duration
	float setpoint;		// record setpoint to produce prop debug
	float aggregate_timer;  // time 5 seconds to record number of drops
} DropDebug;

void check_drop(DropData *drop, MotorData *m, RuntimeData *rt, State *state, DropDebug *drop_dbg);
void drop_deactivate(DropData *drop, DropDebug *drop_dbg, RuntimeData *rt);
void reset_drop(DropData *drop);
void configure_drop(DropData *drop, const tnt_config *config);
void apply_angle_drop(DropData *drop, RuntimeData *rt);
