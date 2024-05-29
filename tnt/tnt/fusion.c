// Copyright 2019 - 2022 Mitch Lustig
// Copyright 2022 Benjamin Vedder <benjamin@vedder.se>
// Copyright 2023 Michael Silberstein
// Copyright 2024 Lukas Hrazky
//
// This file is part of the Trick and Trail VESC package.
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

#include "vesc_c_if.h"

#include "motor_data_tnt.h"
#include "state_tnt.h"
#include "utils_tnt.h"
#include "proportional_gain.h"
#include "traction.h"
#include "surge.h"
#include "runtime.h"
#include "remote_input.h"

#include "conf/datatypes.h"
#include "conf/confparser.h"
#include "conf/confxml.h"
#include "conf/buffer.h"
#include "conf/conf_general.h"

#include <math.h>
#include <string.h>

HEADER

// This is all persistent state of the application, which will be allocated in init. It
// is put here because variables can only be read-only when this program is loaded
// in flash without virtual memory in RAM (as all RAM already is dedicated to the
// main firmware and managed from there). This is probably the main limitation of
// loading applications in runtime, but it is not too bad to work around.
typedef struct {
	lib_thread main_thread;
	fusion_config fusion_conf;

	// Firmware version, passed in from Lisp
	int fw_version_major, fw_version_minor, fw_version_beta;

  	MotorData motor;

	// Config values
	uint32_t loop_time_us;
	float mc_max_temp_fet, mc_max_temp_mot;
	float mc_current_max, mc_current_min;

	// Runtime values grouped for easy access in ancillary functions
	RuntimeData rt; 		// pitch_angle proportional pid_value setpoint current_time roll_angle  last_accel_z  accel[3]

	// Rumtime state values
	State state;

	float disengage_timer, nag_timer;
	float idle_voltage;
	float motor_timeout_s;
	float tb_highvoltage_timer;

	// Odometer
	float odo_timer;
	int odometer_dirty;
	uint64_t odometer;

	//Remote
	RemoteData remote;
	StickyTiltData st_tilt;

	// Feature: Surge
	SurgeData surge;
	SurgeDebug surge_dbg;
	
	//Traction Control
	TractionData traction;
	TractionDebug traction_dbg;

	// Throttle/Brake Scaling
	KpArray accel_kp;
	KpArray brake_kp;

	// Dynamic Stability
	float stabl;
	float stabl_step_size_up, stabl_step_size_down;

	//Trip Debug
	RideTimeData ridetimer;


	//Debug
	float debug1, debug2, debug3, debug4, debug5, debug6;

} data;

static void set_current(data *d, float current);

const VESC_PIN beeper_pin = VESC_PIN_PPM; //BUZZER / BEEPER on Servo Pin


static void configure(data *d) {
	state_init(&d->state, d->fusion_conf.disable_pkg);
	
	// This timer is used to determine how long the board has been disengaged / idle. subtract 1 second to prevent the haptic buzz disengage click on "write config"
	d->disengage_timer = d->rt.current_time - 1;

	// Loop time in microseconds
	d->loop_time_us = 1e6 / d->fusion_conf.hertz;

	// Loop time in seconds times 20 for a nice long grace period
	d->motor_timeout_s = 20.0f / d->fusion_conf.hertz;

	//Dynamic Stability
	d->stabl_step_size_up = 1.0 * d->fusion_conf.stabl_ramp / 100.0 / d->fusion_conf.hertz;
	d->stabl_step_size_down = 1.0 * d->fusion_conf.stabl_ramp_down / 100.0 / d->fusion_conf.hertz;

	d->mc_max_temp_fet = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_fet_start) - 3;
	d->mc_max_temp_mot = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_motor_start) - 3;

	d->mc_current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_current_max);
	// min current is a positive value here!
	d->mc_current_min = fabsf(VESC_IF->get_cfg_float(CFG_PARAM_l_current_min));
	
	//Remote
	configure_remote_features(&d->fusion_conf, &d->remote, &d->st_tilt);

	//Motor Data Configure
	motor_data_configure(&d->motor, 3.0 / d->fusion_conf.hertz);
	
	//initialize current and pitch arrays for acceleration
	angle_kp_reset(&d->accel_kp);
	pitch_kp_configure(&d->fusion_conf, &d->accel_kp, 1);
	
	//initialize current and pitch arrays for braking
	if (d->fusion_conf.brake_curve) {
		angle_kp_reset(&d->brake_kp);
		pitch_kp_configure(&d->fusion_conf, &d->brake_kp, 2);
	}
	
	//Surge Configure
	configure_surge(&d->surge, &d->fusion_conf);

	//Traction Configure
	configure_traction(&d->traction, &d->fusion_conf, &d->traction_dbg);
}

static void reset_vars(data *d) {
	motor_data_reset(&d->motor);
	
	// Set values for startup
	d->brake_timeout = 0;

	//Remote
	reset_remote(&d->remote, &d->st_tilt);

	// Surge
	reset_surge(&d->surge);

	// Traction Control
	reset_traction(&d->traction, &d->state);

	//Stability
	d->stabl = 0;
	
	state_engage(&d->state);
}


/**
 *	check_odometer: see if we need to write back the odometer during fault state
 */
static void check_odometer(data *d)
{
	// Make odometer persistent if we've gone 200m or more
	if (d->odometer_dirty > 0) {
		float stored_odo = VESC_IF->mc_get_odometer();
		if ((stored_odo > d->odometer + 200) || (stored_odo < d->odometer - 10000)) {
			if (d->odometer_dirty == 1) {
				// Wait 10 seconds before writing to avoid writing if immediately continuing to ride
				d->odo_timer = d->rt.current_time;
				d->odometer_dirty++;
			}
			else if ((d->rt.current_time - d->odo_timer) > 10) {
				VESC_IF->store_backup_data();
				d->odometer = VESC_IF->mc_get_odometer();
				d->odometer_dirty = 0;
			}
		}
	}
}

static void set_current(data *d, float current) {
    VESC_IF->timeout_reset();
    VESC_IF->mc_set_current_off_delay(d->motor_timeout_s);
    VESC_IF->mc_set_current(current);
}

static void set_brake(data *d, float current) {
    VESC_IF->timeout_reset();
    VESC_IF->mc_set_current_off_delay(d->motor_timeout_s);
    VESC_IF->mc_set_brake_current(current);
}

static void set_dutycycle(data *d, float dutycycle){
	// Limit duty output to configured max output
	if (dutycycle >  VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty)) {
		dutycycle = VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty);
	} else if(dutycycle < 0 && dutycycle < -VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty)) {
		dutycycle = -VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty);
	}
	
	VESC_IF->timeout_reset();
	VESC_IF->mc_set_current_off_delay(d->motor_timeout_s);
	VESC_IF->mc_set_duty(dutycycle); 
}

static void apply_stability(data *d) {
	float speed_stabl_mod = 0;
	float throttle_stabl_mod = 0;	
	float stabl_mod = 0;
	if (d->fusion_conf.enable_throttle_stability) {
		throttle_stabl_mod = fabsf(d->remote.inputtilt_interpolated) / d->fusion_conf.inputtilt_angle_limit; 	//using inputtilt_interpolated allows the use of sticky tilt and inputtilt smoothing
	}
	if (d->fusion_conf.enable_speed_stability && d->motor.abs_erpm > 1.0 * d->fusion_conf.stabl_min_erpm) {		
		speed_stabl_mod = fminf(1 ,										// Do not exceed the max value.				
				lerp(d->fusion_conf.stabl_min_erpm, d->fusion_conf.stabl_max_erpm, 0, 1, d->motor.abs_erpm));
	}
	stabl_mod = fmaxf(speed_stabl_mod,throttle_stabl_mod);
	float step_size = stabl_mod > d->stabl ? d->stabl_step_size_up : d->stabl_step_size_down;
	rate_limitf(&d->stabl, stabl_mod, step_size); 
}


static void fusion_thd(void *arg) {
	data *d = (data*)arg;

	configure(d);

	while (!VESC_IF->should_terminate()) {

		// Update times
		d->rt.current_time = VESC_IF->system_time();
		if (d->last_time == 0) {
			d->last_time = d->rt.current_time;
		}
		d->diff_time = d->rt.current_time - d->last_time;
		d->last_time = d->rt.current_time;

		motor_data_update(&d->motor);
		update_remote(&d->fusion_conf, &d->remote);

		float new_pid_value = 0;		
		switch(d->state.state) {
		case (STATE_READY):
			d->disengage_timer = d->rt.current_time;

			//Apply Stability
			//if (d->fusion_conf.enable_speed_stability {
			//	apply_stability(d);
			//}

			//Select and Apply Kp
			d->state.braking_pos = sign(d->remote.throttle_val) != d->motor.erpm_sign;
			bool brake_curve = d->fusion_conf.brake_curve && d->state.braking_pos;
			float kp;
			kp = angle_kp_select(d->abs_prop_smooth, 
				brake_curve ? &d->brake_kp : &d->accel_kp);
			d->debug1 = brake_curve ? -kp_mod : kp_mod;
			//kp *= (1 + d->stabl * d->fusion_conf.stabl_pitch_max_scale / 100); //apply dynamic stability
			new_current_demand = kp * d->remote.throttle_val;
			
			
			// Current Limiting
			float current_limit = d->motor.braking ? d->mc_current_min : d->mc_current_max;
			if (fabsf(new_current_demand) > current_limit) {
				new_current_demand = sign(new_current_demand) * current_limit;
			}
			check_current(&d->motor, &d->surge, &d->state, &d->rt,  &d->fusion_conf); // Check for high current conditions
			
			// Modifiers to PID control
			check_traction(&d->motor, &d->traction, &d->state, &d->rt, &d->fusion_conf, &d->traction_dbg);
			if (d->fusion_conf.is_surge_enabled)
				check_surge(&d->motor, &d->surge, &d->state, &d->rt, &d->fusion_conf, &d->surge_dbg);

			// PID value application
			d->rt.current_demand = (d->state.wheelslip && d->fusion_conf.is_traction_enabled) ? 0 : new_current_demand;

			// Output to motor
			if (d->surge.active) { 	
				set_dutycycle(d, d->surge.new_duty_cycle); 		// Set the duty to surge
			} else if (d->state.braking_pos) {
				set_brake(d, d->rt.current_demand);				// Use braking function for traction control
			} else {
				set_current(d, d->rt.current_demand); 			// Set current as normal.
			}

			break;

			check_odometer(d);


			break;
		case (STATE_DISABLED):;
			// no set_current, no brake_current
		default:;
		}

		// Delay between loops
		VESC_IF->sleep_us(d->loop_time_us);
	}
}

static void write_cfg_to_eeprom(data *d) {
	uint32_t ints = sizeof(fusion_config) / 4 + 1;
	uint32_t *buffer = VESC_IF->malloc(ints * sizeof(uint32_t));
	if (!buffer) {
		log_error("Failed to write config to EEPROM: Out of memory.");
		return;
	}
	
	bool write_ok = true;
	memcpy(buffer, &(d->fusion_conf), sizeof(fusion_config));
	for (uint32_t i = 0;i < ints;i++) {
		eeprom_var v;
		v.as_u32 = buffer[i];
		if (!VESC_IF->store_eeprom_var(&v, i + 1)) {
			write_ok = false;
			break;
		}
	}

	VESC_IF->free(buffer);

	if (write_ok) {
		eeprom_var v;
		v.as_u32 = FUSION_CONFIG_SIGNATURE;
		VESC_IF->store_eeprom_var(&v, 0);
	} else {
        	log_error("Failed to write config to EEPROM.");
   	}

	// Emit 1 short beep to confirm writing all settings to eeprom
	beep_alert(d, 1, 0);
}

static void read_cfg_from_eeprom(fusion_config *config) {
	// Read config from EEPROM if signature is correct
	uint32_t ints = sizeof(fusion_config) / 4 + 1;
	uint32_t *buffer = VESC_IF->malloc(ints * sizeof(uint32_t));
	if (!buffer) {
        	log_error("Failed to read config from EEPROM: Out of memory.");
        	return;
    	}
	
	eeprom_var v;	
	bool read_ok = VESC_IF->read_eeprom_var(&v, 0);
	if (read_ok) {
		if (v.as_u32 == FUSION_CONFIG_SIGNATURE) {
			for (uint32_t i = 0;i < ints;i++) {
				if (!VESC_IF->read_eeprom_var(&v, i + 1)) {
					read_ok = false;
					break;
				}
				buffer[i] = v.as_u32;
			}
		} else {
			log_error("Failed signature check while reading config from EEPROM, using defaults.");
			confparser_set_defaults_fusion_config(config);
			return;
	        }
	}

	if (read_ok) {
		memcpy(config, buffer, sizeof(fusion_config));
	} else {
		confparser_set_defaults_fusion_config(config);
		log_error("Failed to read config from EEPROM, using defaults.");
	}

	VESC_IF->free(buffer);
}


static void data_init(data *d) {
    memset(d, 0, sizeof(data));
    read_cfg_from_eeprom(&d->fusion_conf);
    d->odometer = VESC_IF->mc_get_odometer();
}

enum {
    COMMAND_GET_INFO = 0,  // get version / package info
    COMMAND_GET_RTDATA = 1,  // get rt data
    COMMAND_CFG_SAVE = 2,  // save config to eeprom
    COMMAND_CFG_RESTORE = 3,  // restore config from eeprom
} Commands;

static void send_realtime_data(data *d){
	static const int bufsize = 103;
	uint8_t buffer[bufsize];
	int32_t ind = 0;
	buffer[ind++] = 111;//Magic Number
	buffer[ind++] = COMMAND_GET_RTDATA;
	float corr_factor;

	// Board State
	if (d->traction.traction_braking) {
		buffer[ind++] = 5;
	} else if (d->state.wheelslip){
		buffer[ind++] = 4;
	} else { buffer[ind++] = d->state.state; }
	//buffer[ind++] = d->state.wheelslip ? 4 : d->state.state; 
	buffer[ind++] = d->state.sat; 
	buffer[ind++] = (d->footpad_sensor.state & 0xF) + (d->beep_reason << 4);
	buffer[ind++] = d->state.stop_condition;
	buffer_append_float32_auto(buffer, d->footpad_sensor.adc1, &ind);
	buffer_append_float32_auto(buffer, d->footpad_sensor.adc2, &ind);
	buffer_append_float32_auto(buffer, VESC_IF->mc_get_input_voltage_filtered(), &ind);
	buffer_append_float32_auto(buffer, d->motor.current_avg, &ind); // current atr_filtered_current
	buffer_append_float32_auto(buffer, d->rt.pitch_angle, &ind);
	buffer_append_float32_auto(buffer, d->rt.roll_angle, &ind);

	//Tune Modifiers
	buffer_append_float32_auto(buffer, d->rt.setpoint, &ind);
	buffer_append_float32_auto(buffer, d->remote.inputtilt_interpolated, &ind);
	buffer_append_float32_auto(buffer, d->remote.throttle_val, &ind);
	buffer_append_float32_auto(buffer, d->rt.current_time - d->traction.timeron , &ind); //Time since last wheelslip
	buffer_append_float32_auto(buffer, d->rt.current_time - d->surge.timer , &ind); //Time since last surge

	// Trip
	if (d->ridetimer.ride_time > 0) {
		corr_factor =  d->rt.current_time / d->ridetimer.ride_time;
	} else {corr_factor = 1;}
	buffer_append_float32_auto(buffer, d->ridetimer.ride_time, &ind); //Ride Time
	buffer_append_float32_auto(buffer, d->ridetimer.rest_time, &ind); //Rest time
	buffer_append_float32_auto(buffer, VESC_IF->mc_stat_speed_avg() * 3.6 * .621 * corr_factor, &ind); //speed avg convert m/s to mph
	buffer_append_float32_auto(buffer, VESC_IF->mc_stat_current_avg() * corr_factor, &ind); //current avg
	buffer_append_float32_auto(buffer, VESC_IF->mc_stat_power_avg() * corr_factor, &ind); //power avg
	buffer_append_float32_auto(buffer, (VESC_IF->mc_get_watt_hours(false) - VESC_IF->mc_get_watt_hours_charged(false)) / (VESC_IF->mc_get_distance_abs() * 0.000621), &ind); //efficiency
	
	// DEBUG
	if (d->fusion_conf.is_tcdebug_enabled) {
		buffer[ind++] = 1;
		buffer_append_float32_auto(buffer, d->traction_dbg.debug2, &ind); //wheelslip erpm factor
		buffer_append_float32_auto(buffer, d->traction_dbg.debug6, &ind); //accel at wheelslip start
		buffer_append_float32_auto(buffer, d->traction_dbg.debug7, &ind); //time to reduce accel //erpm before wheel slip debug3
		buffer_append_float32_auto(buffer, d->traction_dbg.debug9, &ind); //erpm at wheel slip
		buffer_append_float32_auto(buffer, d->traction_dbg.debug4, &ind); //Debug condition or last accel d->traction_dbg.debug4
		buffer_append_float32_auto(buffer, d->traction_dbg.debug8, &ind); //duration
		buffer_append_float32_auto(buffer, d->traction_dbg.debug5, &ind); //count 
	} else if (d->fusion_conf.is_surgedebug_enabled) {
		buffer[ind++] = 2;
		buffer_append_float32_auto(buffer, d->surge_dbg.debug1, &ind); //surge start proportional
		buffer_append_float32_auto(buffer, d->surge_dbg.debug5, &ind); //surge added duty cycle
		buffer_append_float32_auto(buffer, d->surge_dbg.debug3, &ind); //surge start current threshold
		buffer_append_float32_auto(buffer, d->surge_dbg.debug6, &ind); //surge end 
		buffer_append_float32_auto(buffer, d->surge_dbg.debug7, &ind); //Duration last surge cycle time
		buffer_append_float32_auto(buffer, d->surge_dbg.debug2, &ind); //start current value
		buffer_append_float32_auto(buffer, d->surge_dbg.debug8, &ind); //ramp rate
	} else if (d->fusion_conf.is_tunedebug_enabled) {
		buffer[ind++] = 3;
		buffer_append_float32_auto(buffer, d->pitch_smooth_kalman, &ind); //smooth pitch	
		buffer_append_float32_auto(buffer, d->debug1, &ind); // scaled angle P
		buffer_append_float32_auto(buffer, d->debug1*d->stabl*d->fusion_conf.stabl_pitch_max_scale/100.0, &ind); // added stiffnes pitch kp
		buffer_append_float32_auto(buffer, d->debug3, &ind); // added stability rate P
		buffer_append_float32_auto(buffer, d->stabl, &ind);
		buffer_append_float32_auto(buffer, d->debug2, &ind); //rollkp d->debug2
	} else if (d->fusion_conf.is_yawdebug_enabled) {
		buffer[ind++] = 4;
		buffer_append_float32_auto(buffer, d->yaw_angle, &ind); //yaw angle
		buffer_append_float32_auto(buffer, d->yaw_dbg.debug1 * d->fusion_conf.hertz, &ind); //yaw change
		buffer_append_float32_auto(buffer, d->yaw_dbg.debug3, &ind); //yaw kp raw
		buffer_append_float32_auto(buffer, d->yaw_dbg.debug4, &ind); //yaw kp scaled	
		buffer_append_float32_auto(buffer, d->yaw_dbg.debug5, &ind); //erpm scaler
		buffer_append_float32_auto(buffer, d->yaw_dbg.debug2, &ind); //max kp change
	} else { 
		buffer[ind++] = 0; 
	}

	SEND_APP_DATA(buffer, bufsize, ind);
}

// Handler for incoming app commands
static void on_command_received(unsigned char *buffer, unsigned int len) {
	data *d = (data*)ARG;
	uint8_t magicnr = buffer[0];
	uint8_t command = buffer[1];

	if(len < 2){
		log_error("Received command data too short.");
		return;
	}
	if (magicnr != 111) {
		log_error("Invalid Package ID: %u", magicnr);
		return;
	}

	switch(command) {
		case COMMAND_GET_INFO: {
			int32_t ind = 0;
			uint8_t buffer[10];
			buffer[ind++] = 111;	// magic nr.
			buffer[ind++] = 0x0;	// command ID
			buffer[ind++] = (uint8_t) (10 * APPCONF_TNT_VERSION);
			buffer[ind++] = 1;
			VESC_IF->send_app_data(buffer, ind);
			return;
		}
		case COMMAND_GET_RTDATA: {
			send_realtime_data(d);
			return;
		}
		case COMMAND_CFG_RESTORE: {
			read_cfg_from_eeprom(&d->fusion_conf);
			return;
		}
		case COMMAND_CFG_SAVE: {
			write_cfg_to_eeprom(d);
			return;
		}
		default: {
			if (!VESC_IF->app_is_output_disabled()) {
				log_error("Unknown command received: %u", command);
			}
		}
	}
}

// Called from Lisp on init to pass in the version info of the firmware
static lbm_value ext_set_fw_version(lbm_value *args, lbm_uint argn) {
	data *d = (data*)ARG;
	if (argn > 2) {
		d->fw_version_major = VESC_IF->lbm_dec_as_i32(args[0]);
		d->fw_version_minor = VESC_IF->lbm_dec_as_i32(args[1]);
		d->fw_version_beta = VESC_IF->lbm_dec_as_i32(args[2]);
	}
	return VESC_IF->lbm_enc_sym_true;
}

// These functions are used to send the config page to VESC Tool
// and to make persistent read and write work
static int get_cfg(uint8_t *buffer, bool is_default) {
	data *d = (data *) ARG;

	fusion_config *cfg;
	if (is_default) {
		cfg = VESC_IF->malloc(sizeof(fusion_config));
		if (!cfg) {
			log_error("Failed to send default config to VESC tool: Out of memory.");
			return 0;
		}
		confparser_set_defaults_fusion_config(cfg);
	} else {
		cfg = &d->fusion_conf;
	}

	int res = confparser_serialize_fusion_config(buffer, cfg);

	if (is_default) {
		VESC_IF->free(cfg);
	}

	return res;
}

static bool set_cfg(uint8_t *buffer) {
	data *d = (data *) ARG;

	// don't let users use the TNT Cfg "write" button in special modes
	if (d->state.mode != MODE_NORMAL) {
		return false;
	}
	
	bool res = confparser_deserialize_fusion_config(buffer, &d->fusion_conf);
	
	// Store to EEPROM
	if (res) {
		write_cfg_to_eeprom(d);
		configure(d);
	}
	
	return res;
}

static int get_cfg_xml(uint8_t **buffer) {
	// Note: As the address of data_fusion_config_ is not known
	// at compile time it will be relative to where it is in the
	// linked binary. Therefore we add PROG_ADDR to it so that it
	// points to where it ends up on the STM32.
	*buffer = data_fusion_config_ + PROG_ADDR;
	return DATA_FUSION_CONFIG__SIZE;
}

// Called when code is stopped
static void stop(void *arg) {
	data *d = (data *) arg;
	VESC_IF->set_app_data_handler(NULL);
	VESC_IF->conf_custom_clear_configs();
	VESC_IF->request_terminate(d->main_thread);
	log_msg("Terminating.");
	VESC_IF->free(d);
}

INIT_FUN(lib_info *info) {
	INIT_START
	VESC_IF->printf("Init TNT v%.1fd\n", (double)APPCONF_TNT_VERSION);
	
	data *d = VESC_IF->malloc(sizeof(data));
	if (!d) {
		log_error("Out of memory, startup failed.");
		return false;
	}
	data_init(d);

	info->stop_fun = stop;	
	info->arg = d;
	
	VESC_IF->conf_custom_add_config(get_cfg, set_cfg, get_cfg_xml)

	d->main_thread = VESC_IF->spawn(fusion_thd, 1024, "Fusion Main", d);
	if (!d->main_thread) {
		log_error("Failed to spawn Fusion Main thread.");
		return false;
	}

	VESC_IF->set_app_data_handler(on_command_received);
	VESC_IF->lbm_add_extension("ext-set-fw-version", ext_set_fw_version);

	return true;
}

void send_app_data_overflow_terminate() {
    VESC_IF->request_terminate(((data *) ARG)->main_thread);
}
