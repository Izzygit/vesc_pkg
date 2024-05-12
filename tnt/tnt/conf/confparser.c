// This file is autogenerated by VESC Tool

#include <string.h>
#include "buffer.h"
#include "conf_general.h"
#include "confparser.h"

int32_t confparser_serialize_tnt_config(uint8_t *buffer, const tnt_config *conf) {
	int32_t ind = 0;

	buffer_append_uint32(buffer, TNT_CONFIG_SIGNATURE, &ind);

	buffer_append_float16(buffer, conf->kp0, 1, &ind);
	buffer_append_float16(buffer, conf->current1, 10, &ind);
	buffer_append_float16(buffer, conf->current2, 10, &ind);
	buffer_append_float16(buffer, conf->current3, 10, &ind);
	buffer_append_float16(buffer, conf->current4, 10, &ind);
	buffer_append_float16(buffer, conf->current5, 10, &ind);
	buffer_append_float16(buffer, conf->pitch1, 100, &ind);
	buffer_append_float16(buffer, conf->pitch2, 100, &ind);
	buffer_append_float16(buffer, conf->pitch3, 10, &ind);
	buffer_append_float16(buffer, conf->pitch4, 10, &ind);
	buffer_append_float16(buffer, conf->pitch5, 10, &ind);
	buffer[ind++] = conf->pitch_kp_input;
	buffer_append_float16(buffer, conf->mahony_kp, 100, &ind);
	buffer_append_float16(buffer, conf->kp_rate, 100, &ind);
	buffer_append_float16(buffer, conf->pitch_filter, 10, &ind);
	buffer_append_float16(buffer, conf->kalman_factor1, 100, &ind);
	buffer_append_float16(buffer, conf->kalman_factor2, 100, &ind);
	buffer_append_float16(buffer, conf->kalman_factor3, 100, &ind);
	buffer[ind++] = conf->brake_curve;
	buffer_append_float16(buffer, conf->brake_kp0, 1, &ind);
	buffer_append_float16(buffer, conf->brakekp_rate, 100, &ind);
	buffer_append_float16(buffer, conf->brakecurrent1, 10, &ind);
	buffer_append_float16(buffer, conf->brakecurrent2, 10, &ind);
	buffer_append_float16(buffer, conf->brakecurrent3, 10, &ind);
	buffer_append_float16(buffer, conf->brakepitch1, 100, &ind);
	buffer_append_float16(buffer, conf->brakepitch2, 100, &ind);
	buffer_append_float16(buffer, conf->brakepitch3, 10, &ind);
	buffer_append_float16(buffer, conf->roll_kp1, 100, &ind);
	buffer_append_float16(buffer, conf->roll_kp2, 100, &ind);
	buffer_append_float16(buffer, conf->roll_kp3, 100, &ind);
	buffer_append_float16(buffer, conf->roll1, 10, &ind);
	buffer_append_float16(buffer, conf->roll2, 10, &ind);
	buffer_append_float16(buffer, conf->roll3, 10, &ind);
	buffer_append_float16(buffer, conf->brkroll_kp1, 100, &ind);
	buffer_append_float16(buffer, conf->brkroll_kp2, 100, &ind);
	buffer_append_float16(buffer, conf->brkroll_kp3, 100, &ind);
	buffer_append_float16(buffer, conf->brkroll1, 10, &ind);
	buffer_append_float16(buffer, conf->brkroll2, 10, &ind);
	buffer_append_float16(buffer, conf->brkroll3, 10, &ind);
	buffer_append_float16(buffer, conf->rollkp_higherpm, 1, &ind);
	buffer_append_float16(buffer, conf->rollkp_lowerpm, 1, &ind);
	buffer_append_float16(buffer, conf->rollkp_maxscale, 1, &ind);
	buffer_append_float16(buffer, conf->yaw_kp1, 100, &ind);
	buffer_append_float16(buffer, conf->yaw_kp2, 100, &ind);
	buffer_append_float16(buffer, conf->yaw_kp3, 100, &ind);
	buffer_append_float16(buffer, conf->yaw1, 1, &ind);
	buffer_append_float16(buffer, conf->yaw2, 1, &ind);
	buffer_append_float16(buffer, conf->yaw3, 1, &ind);
	buffer_append_float16(buffer, conf->brkyaw_kp1, 100, &ind);
	buffer_append_float16(buffer, conf->brkyaw_kp2, 100, &ind);
	buffer_append_float16(buffer, conf->brkyaw_kp3, 100, &ind);
	buffer_append_float16(buffer, conf->brkyaw1, 1, &ind);
	buffer_append_float16(buffer, conf->brkyaw2, 1, &ind);
	buffer_append_float16(buffer, conf->brkyaw3, 1, &ind);
	buffer_append_float16(buffer, conf->yaw_minerpm, 1, &ind);
	buffer[ind++] = conf->is_surge_enabled;
	buffer_append_float16(buffer, conf->surge_startcurrent, 10, &ind);
	buffer_append_float16(buffer, conf->surge_start_hd_current, 10, &ind);
	buffer_append_float16(buffer, conf->surge_scaleduty, 1, &ind);
	buffer_append_float16(buffer, conf->surge_pitchmargin, 10, &ind);
	buffer_append_float16(buffer, conf->surge_maxangle, 10, &ind);
	buffer_append_float16(buffer, conf->surge_minerpm, 1, &ind);
	buffer_append_float16(buffer, conf->surge_duty, 1, &ind);
	buffer_append_float16(buffer, conf->tiltback_surge_speed, 1, &ind);
	buffer[ind++] = conf->is_traction_enabled;
	buffer_append_float16(buffer, conf->wheelslip_margin, 1, &ind);
	buffer_append_float16(buffer, conf->wheelslip_accelstart, 1, &ind);
	buffer_append_float16(buffer, conf->wheelslip_accelend, 1, &ind);
	buffer_append_float16(buffer, conf->wheelslip_scaleaccel, 10, &ind);
	buffer_append_float16(buffer, conf->wheelslip_scaleerpm, 1, &ind);
	buffer[ind++] = conf->enable_speed_stability;
	buffer[ind++] = conf->enable_throttle_stability;
	buffer_append_float16(buffer, conf->stabl_pitch_max_scale, 1, &ind);
	buffer_append_float16(buffer, conf->stabl_rate_max_scale, 1, &ind);
	buffer_append_float16(buffer, conf->stabl_min_erpm, 1, &ind);
	buffer_append_float16(buffer, conf->stabl_max_erpm, 1, &ind);
	buffer_append_float16(buffer, conf->stabl_ramp, 1, &ind);
	buffer_append_float16(buffer, conf->stabl_ramp_down, 1, &ind);
	buffer_append_uint16(buffer, conf->hertz, &ind);
	buffer_append_float16(buffer, conf->fault_pitch, 10, &ind);
	buffer_append_float16(buffer, conf->fault_roll, 10, &ind);
	buffer_append_float16(buffer, conf->fault_adc1, 1000, &ind);
	buffer_append_float16(buffer, conf->fault_adc2, 1000, &ind);
	buffer[ind++] = conf->is_footbeep_enabled;
	buffer_append_uint16(buffer, conf->fault_delay_pitch, &ind);
	buffer_append_uint16(buffer, conf->fault_delay_switch_half, &ind);
	buffer_append_uint16(buffer, conf->fault_delay_switch_full, &ind);
	buffer_append_uint16(buffer, conf->fault_adc_half_erpm, &ind);
	buffer[ind++] = conf->fault_is_dual_switch;
	buffer[ind++] = conf->fault_moving_fault_disabled;
	buffer_append_float16(buffer, conf->tiltback_duty_angle, 100, &ind);
	buffer_append_float16(buffer, conf->tiltback_duty_speed, 100, &ind);
	buffer_append_float16(buffer, conf->tiltback_duty, 1000, &ind);
	buffer[ind++] = conf->is_dutybeep_enabled;
	buffer_append_float16(buffer, conf->tiltback_hv_angle, 100, &ind);
	buffer_append_float16(buffer, conf->tiltback_hv_speed, 100, &ind);
	buffer_append_float16(buffer, conf->tiltback_hv, 10, &ind);
	buffer_append_float16(buffer, conf->tiltback_lv_angle, 100, &ind);
	buffer_append_float16(buffer, conf->tiltback_lv_speed, 100, &ind);
	buffer_append_float16(buffer, conf->tiltback_lv, 10, &ind);
	buffer_append_float16(buffer, conf->tiltback_ht_angle, 100, &ind);
	buffer_append_float16(buffer, conf->tiltback_ht_speed, 100, &ind);
	buffer_append_float16(buffer, conf->tiltback_return_speed, 100, &ind);
	buffer_append_float16(buffer, conf->tiltback_constant, 1000, &ind);
	buffer_append_uint16(buffer, conf->tiltback_constant_erpm, &ind);
	buffer[ind++] = (uint8_t)conf->haptic_buzz_intensity;
	buffer[ind++] = (uint8_t)conf->haptic_buzz_min;
	buffer[ind++] = conf->haptic_buzz_current;
	buffer[ind++] = conf->haptic_buzz_duty;
	buffer_append_float16(buffer, conf->noseangling_speed, 100, &ind);
	buffer[ind++] = conf->inputtilt_remote_type;
	buffer_append_float16(buffer, conf->inputtilt_angle_limit, 100, &ind);
	buffer_append_float16(buffer, conf->inputtilt_speed, 1, &ind);
	buffer[ind++] = (uint8_t)conf->inputtilt_smoothing_factor;
	buffer[ind++] = conf->inputtilt_invert_throttle;
	buffer_append_float16(buffer, conf->inputtilt_deadband, 10000, &ind);
	buffer_append_float16(buffer, conf->stickytiltval1, 10, &ind);
	buffer_append_float16(buffer, conf->stickytiltval2, 10, &ind);
	buffer_append_uint16(buffer, conf->stickytilt_holdcurrent, &ind);
	buffer[ind++] = conf->is_stickytilt_enabled;
	buffer_append_float16(buffer, conf->startup_pitch_tolerance, 10, &ind);
	buffer_append_float16(buffer, conf->startup_speed, 10, &ind);
	buffer[ind++] = conf->startup_simplestart_enabled;
	buffer[ind++] = conf->startup_pushstart_enabled;
	buffer[ind++] = conf->startup_dirtylandings_enabled;
	buffer_append_float16(buffer, conf->brake_current, 1, &ind);
	buffer_append_int16(buffer, conf->overcurrent_margin, &ind);
	buffer_append_float16(buffer, conf->overcurrent_period, 10, &ind);
	buffer[ind++] = conf->is_beeper_enabled;
	buffer[ind++] = conf->is_surgedebug_enabled;
	buffer[ind++] = conf->is_tcdebug_enabled;
	buffer[ind++] = conf->is_tunedebug_enabled;
	buffer[ind++] = conf->is_yawdebug_enabled;
	buffer[ind++] = conf->disable_pkg;
	buffer_append_float16(buffer, conf->version, 1000, &ind);

	return ind;
}

bool confparser_deserialize_tnt_config(const uint8_t *buffer, tnt_config *conf) {
	int32_t ind = 0;

	uint32_t signature = buffer_get_uint32(buffer, &ind);
	if (signature != TNT_CONFIG_SIGNATURE) {
		return false;
	}

	conf->kp0 = buffer_get_float16(buffer, 1, &ind);
	conf->current1 = buffer_get_float16(buffer, 10, &ind);
	conf->current2 = buffer_get_float16(buffer, 10, &ind);
	conf->current3 = buffer_get_float16(buffer, 10, &ind);
	conf->current4 = buffer_get_float16(buffer, 10, &ind);
	conf->current5 = buffer_get_float16(buffer, 10, &ind);
	conf->pitch1 = buffer_get_float16(buffer, 100, &ind);
	conf->pitch2 = buffer_get_float16(buffer, 100, &ind);
	conf->pitch3 = buffer_get_float16(buffer, 10, &ind);
	conf->pitch4 = buffer_get_float16(buffer, 10, &ind);
	conf->pitch5 = buffer_get_float16(buffer, 10, &ind);
	conf->pitch_kp_input = buffer[ind++];
	conf->mahony_kp = buffer_get_float16(buffer, 100, &ind);
	conf->kp_rate = buffer_get_float16(buffer, 100, &ind);
	conf->pitch_filter = buffer_get_float16(buffer, 10, &ind);
	conf->kalman_factor1 = buffer_get_float16(buffer, 100, &ind);
	conf->kalman_factor2 = buffer_get_float16(buffer, 100, &ind);
	conf->kalman_factor3 = buffer_get_float16(buffer, 100, &ind);
	conf->brake_curve = buffer[ind++];
	conf->brake_kp0 = buffer_get_float16(buffer, 1, &ind);
	conf->brakekp_rate = buffer_get_float16(buffer, 100, &ind);
	conf->brakecurrent1 = buffer_get_float16(buffer, 10, &ind);
	conf->brakecurrent2 = buffer_get_float16(buffer, 10, &ind);
	conf->brakecurrent3 = buffer_get_float16(buffer, 10, &ind);
	conf->brakepitch1 = buffer_get_float16(buffer, 100, &ind);
	conf->brakepitch2 = buffer_get_float16(buffer, 100, &ind);
	conf->brakepitch3 = buffer_get_float16(buffer, 10, &ind);
	conf->roll_kp1 = buffer_get_float16(buffer, 100, &ind);
	conf->roll_kp2 = buffer_get_float16(buffer, 100, &ind);
	conf->roll_kp3 = buffer_get_float16(buffer, 100, &ind);
	conf->roll1 = buffer_get_float16(buffer, 10, &ind);
	conf->roll2 = buffer_get_float16(buffer, 10, &ind);
	conf->roll3 = buffer_get_float16(buffer, 10, &ind);
	conf->brkroll_kp1 = buffer_get_float16(buffer, 100, &ind);
	conf->brkroll_kp2 = buffer_get_float16(buffer, 100, &ind);
	conf->brkroll_kp3 = buffer_get_float16(buffer, 100, &ind);
	conf->brkroll1 = buffer_get_float16(buffer, 10, &ind);
	conf->brkroll2 = buffer_get_float16(buffer, 10, &ind);
	conf->brkroll3 = buffer_get_float16(buffer, 10, &ind);
	conf->rollkp_higherpm = buffer_get_float16(buffer, 1, &ind);
	conf->rollkp_lowerpm = buffer_get_float16(buffer, 1, &ind);
	conf->rollkp_maxscale = buffer_get_float16(buffer, 1, &ind);
	conf->yaw_kp1 = buffer_get_float16(buffer, 100, &ind);
	conf->yaw_kp2 = buffer_get_float16(buffer, 100, &ind);
	conf->yaw_kp3 = buffer_get_float16(buffer, 100, &ind);
	conf->yaw1 = buffer_get_float16(buffer, 1, &ind);
	conf->yaw2 = buffer_get_float16(buffer, 1, &ind);
	conf->yaw3 = buffer_get_float16(buffer, 1, &ind);
	conf->brkyaw_kp1 = buffer_get_float16(buffer, 100, &ind);
	conf->brkyaw_kp2 = buffer_get_float16(buffer, 100, &ind);
	conf->brkyaw_kp3 = buffer_get_float16(buffer, 100, &ind);
	conf->brkyaw1 = buffer_get_float16(buffer, 1, &ind);
	conf->brkyaw2 = buffer_get_float16(buffer, 1, &ind);
	conf->brkyaw3 = buffer_get_float16(buffer, 1, &ind);
	conf->yaw_minerpm = buffer_get_float16(buffer, 1, &ind);
	conf->is_surge_enabled = buffer[ind++];
	conf->surge_startcurrent = buffer_get_float16(buffer, 10, &ind);
	conf->surge_start_hd_current = buffer_get_float16(buffer, 10, &ind);
	conf->surge_scaleduty = buffer_get_float16(buffer, 1, &ind);
	conf->surge_pitchmargin = buffer_get_float16(buffer, 10, &ind);
	conf->surge_maxangle = buffer_get_float16(buffer, 10, &ind);
	conf->surge_minerpm = buffer_get_float16(buffer, 1, &ind);
	conf->surge_duty = buffer_get_float16(buffer, 1, &ind);
	conf->tiltback_surge_speed = buffer_get_float16(buffer, 1, &ind);
	conf->is_traction_enabled = buffer[ind++];
	conf->wheelslip_margin = buffer_get_float16(buffer, 1, &ind);
	conf->wheelslip_accelstart = buffer_get_float16(buffer, 1, &ind);
	conf->wheelslip_accelend = buffer_get_float16(buffer, 1, &ind);
	conf->wheelslip_scaleaccel = buffer_get_float16(buffer, 10, &ind);
	conf->wheelslip_scaleerpm = buffer_get_float16(buffer, 1, &ind);
	conf->enable_speed_stability = buffer[ind++];
	conf->enable_throttle_stability = buffer[ind++];
	conf->stabl_pitch_max_scale = buffer_get_float16(buffer, 1, &ind);
	conf->stabl_rate_max_scale = buffer_get_float16(buffer, 1, &ind);
	conf->stabl_min_erpm = buffer_get_float16(buffer, 1, &ind);
	conf->stabl_max_erpm = buffer_get_float16(buffer, 1, &ind);
	conf->stabl_ramp = buffer_get_float16(buffer, 1, &ind);
	conf->stabl_ramp_down = buffer_get_float16(buffer, 1, &ind);
	conf->hertz = buffer_get_uint16(buffer, &ind);
	conf->fault_pitch = buffer_get_float16(buffer, 10, &ind);
	conf->fault_roll = buffer_get_float16(buffer, 10, &ind);
	conf->fault_adc1 = buffer_get_float16(buffer, 1000, &ind);
	conf->fault_adc2 = buffer_get_float16(buffer, 1000, &ind);
	conf->is_footbeep_enabled = buffer[ind++];
	conf->fault_delay_pitch = buffer_get_uint16(buffer, &ind);
	conf->fault_delay_switch_half = buffer_get_uint16(buffer, &ind);
	conf->fault_delay_switch_full = buffer_get_uint16(buffer, &ind);
	conf->fault_adc_half_erpm = buffer_get_uint16(buffer, &ind);
	conf->fault_is_dual_switch = buffer[ind++];
	conf->fault_moving_fault_disabled = buffer[ind++];
	conf->tiltback_duty_angle = buffer_get_float16(buffer, 100, &ind);
	conf->tiltback_duty_speed = buffer_get_float16(buffer, 100, &ind);
	conf->tiltback_duty = buffer_get_float16(buffer, 1000, &ind);
	conf->is_dutybeep_enabled = buffer[ind++];
	conf->tiltback_hv_angle = buffer_get_float16(buffer, 100, &ind);
	conf->tiltback_hv_speed = buffer_get_float16(buffer, 100, &ind);
	conf->tiltback_hv = buffer_get_float16(buffer, 10, &ind);
	conf->tiltback_lv_angle = buffer_get_float16(buffer, 100, &ind);
	conf->tiltback_lv_speed = buffer_get_float16(buffer, 100, &ind);
	conf->tiltback_lv = buffer_get_float16(buffer, 10, &ind);
	conf->tiltback_ht_angle = buffer_get_float16(buffer, 100, &ind);
	conf->tiltback_ht_speed = buffer_get_float16(buffer, 100, &ind);
	conf->tiltback_return_speed = buffer_get_float16(buffer, 100, &ind);
	conf->tiltback_constant = buffer_get_float16(buffer, 1000, &ind);
	conf->tiltback_constant_erpm = buffer_get_uint16(buffer, &ind);
	conf->haptic_buzz_intensity = buffer[ind++];
	conf->haptic_buzz_min = buffer[ind++];
	conf->haptic_buzz_current = buffer[ind++];
	conf->haptic_buzz_duty = buffer[ind++];
	conf->noseangling_speed = buffer_get_float16(buffer, 100, &ind);
	conf->inputtilt_remote_type = buffer[ind++];
	conf->inputtilt_angle_limit = buffer_get_float16(buffer, 100, &ind);
	conf->inputtilt_speed = buffer_get_float16(buffer, 1, &ind);
	conf->inputtilt_smoothing_factor = buffer[ind++];
	conf->inputtilt_invert_throttle = buffer[ind++];
	conf->inputtilt_deadband = buffer_get_float16(buffer, 10000, &ind);
	conf->stickytiltval1 = buffer_get_float16(buffer, 10, &ind);
	conf->stickytiltval2 = buffer_get_float16(buffer, 10, &ind);
	conf->stickytilt_holdcurrent = buffer_get_uint16(buffer, &ind);
	conf->is_stickytilt_enabled = buffer[ind++];
	conf->startup_pitch_tolerance = buffer_get_float16(buffer, 10, &ind);
	conf->startup_speed = buffer_get_float16(buffer, 10, &ind);
	conf->startup_simplestart_enabled = buffer[ind++];
	conf->startup_pushstart_enabled = buffer[ind++];
	conf->startup_dirtylandings_enabled = buffer[ind++];
	conf->brake_current = buffer_get_float16(buffer, 1, &ind);
	conf->overcurrent_margin = buffer_get_int16(buffer, &ind);
	conf->overcurrent_period = buffer_get_float16(buffer, 10, &ind);
	conf->is_beeper_enabled = buffer[ind++];
	conf->is_surgedebug_enabled = buffer[ind++];
	conf->is_tcdebug_enabled = buffer[ind++];
	conf->is_tunedebug_enabled = buffer[ind++];
	conf->is_yawdebug_enabled = buffer[ind++];
	conf->disable_pkg = buffer[ind++];
	conf->version = buffer_get_float16(buffer, 1000, &ind);

	return true;
}

void confparser_set_defaults_tnt_config(tnt_config *conf) {
	conf->kp0 = APPCONF_TNT_KP0;
	conf->current1 = APPCONF_TNT_CURRENT1;
	conf->current2 = APPCONF_TNT_CURRENT2;
	conf->current3 = APPCONF_TNT_CURRENT3;
	conf->current4 = APPCONF_TNT_CURRENT4;
	conf->current5 = APPCONF_TNT_CURRENT5;
	conf->pitch1 = APPCONF_TNT_PITCH1;
	conf->pitch2 = APPCONF_TNT_PITCH2;
	conf->pitch3 = APPCONF_TNT_PITCH3;
	conf->pitch4 = APPCONF_TNT_PITCH4;
	conf->pitch5 = APPCONF_TNT_PITCH5;
	conf->pitch_kp_input = APPCONF_TNT_PITCH_KP_INPUT;
	conf->mahony_kp = APPCONF_TNT_MAHONY_KP;
	conf->kp_rate = APPCONF_TNT_KP_RATE;
	conf->pitch_filter = APPCONF_TNT_PITCH_FILTER;
	conf->kalman_factor1 = APPCONF_TNT_KALMAN1;
	conf->kalman_factor2 = APPCONF_TNT_KALMAN2;
	conf->kalman_factor3 = APPCONF_TNT_KALMAN3;
	conf->brake_curve = APPCONF_TNT_BRAKE_CURVE;
	conf->brake_kp0 = APPCONF_TNT_BRAKEKP0;
	conf->brakekp_rate = APPCONF_TNT_BRAKEKP_RATE;
	conf->brakecurrent1 = APPCONF_TNT_BRAKECURRENT1;
	conf->brakecurrent2 = APPCONF_TNT_BRAKECURRENT2;
	conf->brakecurrent3 = APPCONF_TNT_BRAKECURRENT3;
	conf->brakepitch1 = APPCONF_TNT_BRAKEPITCH1;
	conf->brakepitch2 = APPCONF_TNT_BRAKEPITCH2;
	conf->brakepitch3 = APPCONF_TNT_BRAKEPITCH3;
	conf->roll_kp1 = APPCONF_TNT_ROLL_KP1;
	conf->roll_kp2 = APPCONF_TNT_ROLL_KP2;
	conf->roll_kp3 = APPCONF_TNT_ROLL_KP3;
	conf->roll1 = APPCONF_TNT_ROLL1;
	conf->roll2 = APPCONF_TNT_ROLL2;
	conf->roll3 = APPCONF_TNT_ROLL3;
	conf->brkroll_kp1 = APPCONF_TNT_BRKROLL_KP1;
	conf->brkroll_kp2 = APPCONF_TNT_BRKROLL_KP2;
	conf->brkroll_kp3 = APPCONF_TNT_BRKROLL_KP3;
	conf->brkroll1 = APPCONF_TNT_BRKROLL1;
	conf->brkroll2 = APPCONF_TNT_BRKROLL2;
	conf->brkroll3 = APPCONF_TNT_BRKROLL3;
	conf->rollkp_higherpm = APPCONF_TNT_ROLLKP_HIGHERPM;
	conf->rollkp_lowerpm = APPCONF_TNT_ROLLKP_LOWERPM;
	conf->rollkp_maxscale = APPCONF_TNT_ROLLKP_MAXSCALE;
	conf->yaw_kp1 = APPCONF_TNT_YAW_KP1;
	conf->yaw_kp2 = APPCONF_TNT_YAW_KP2;
	conf->yaw_kp3 = APPCONF_TNT_YAW_KP3;
	conf->yaw1 = APPCONF_TNT_YAW1;
	conf->yaw2 = APPCONF_TNT_YAW2;
	conf->yaw3 = APPCONF_TNT_YAW3;
	conf->brkyaw_kp1 = APPCONF_TNT_BRKYAW_KP1;
	conf->brkyaw_kp2 = APPCONF_TNT_BRKYAW_KP2;
	conf->brkyaw_kp3 = APPCONF_TNT_BRKYAW_KP3;
	conf->brkyaw1 = APPCONF_TNT_BRKYAW1;
	conf->brkyaw2 = APPCONF_TNT_BRKYAW2;
	conf->brkyaw3 = APPCONF_TNT_BRKYAW3;
	conf->yaw_minerpm = APPCONF_TNT_YAW_MINERPM;
	conf->is_surge_enabled = APPCONF_TNT_IS_SURGE_ENABLED;
	conf->surge_startcurrent = APPCONF_TNT_SURGE_STARTCURRENT;
	conf->surge_start_hd_current = APPCONF_TNT_SURGE_START_HD_CURRENT;
	conf->surge_scaleduty = APPCONF_TNT_SURGE_SCALEDUTY;
	conf->surge_pitchmargin = APPCONF_TNT_SURGE_PITCHMARGIN;
	conf->surge_maxangle = APPCONF_TNT_SURGE_MAXANGLE;
	conf->surge_minerpm = APPCONF_TNT_SURGE_MINERPM;
	conf->surge_duty = APPCONF_TNT_SURGE_DUTY;
	conf->tiltback_surge_speed = APPCONF_TNT_TILTBACK_SURGE_SPEED;
	conf->is_traction_enabled = APPCONF_TNT_IS_TRACTION_ENABLED;
	conf->wheelslip_margin = APPCONF_TNT_WHEELSLIP_MARGIN;
	conf->wheelslip_accelstart = APPCONF_TNT_WHEELSLIP_ACCELSTART;
	conf->wheelslip_accelend = APPCONF_TNT_WHEELSLIP_ACCELEND;
	conf->wheelslip_scaleaccel = APPCONF_TNT_WHEELSLIP_SCALEACCEL;
	conf->wheelslip_scaleerpm = APPCONF_TNT_WHEELSLIP_SCALEERPM;
	conf->enable_speed_stability = APPCONF_TNT_ENABLE_SPEED_STABILITY;
	conf->enable_throttle_stability = APPCONF_TNT_ENABLE_THROTTLE_STABILITY;
	conf->stabl_pitch_max_scale = APPCONF_TNT_STABL_PITCH_MAXSCALE;
	conf->stabl_rate_max_scale = APPCONF_TNT_STABL_RATE_MAXSCALE;
	conf->stabl_min_erpm = APPCONF_TNT_STABL_MIN_ERPM;
	conf->stabl_max_erpm = APPCONF_TNT_STABL_MAX_ERPM;
	conf->stabl_ramp = APPCONF_TNT_STABL_RAMP;
	conf->stabl_ramp_down = APPCONF_TNT_STABL_RAMP_DOWN;
	conf->hertz = APPCONF_TNT_HERTZ;
	conf->fault_pitch = APPCONF_TNT_FAULT_PITCH;
	conf->fault_roll = APPCONF_TNT_FAULT_ROLL;
	conf->fault_adc1 = APPCONF_TNT_FAULT_ADC1;
	conf->fault_adc2 = APPCONF_TNT_FAULT_ADC2;
	conf->is_footbeep_enabled = APPCONF_TNT_IS_FOOTBEEP_ENABLED;
	conf->fault_delay_pitch = APPCONF_TNT_FAULT_DELAY_PITCH;
	conf->fault_delay_switch_half = APPCONF_TNT_FAULT_DELAY_SWITCH_HALF;
	conf->fault_delay_switch_full = APPCONF_TNT_FAULT_DELAY_SWITCH_FULL;
	conf->fault_adc_half_erpm = APPCONF_TNT_FAULT_ADC_HALF_ERPM;
	conf->fault_is_dual_switch = APPCONF_TNT_FAULT_IS_DUAL_SWITCH;
	conf->fault_moving_fault_disabled = APPCONF_TNT_FAULT_MOVING_FAULT_DISABLED;
	conf->tiltback_duty_angle = APPCONF_TNT_TILTBACK_DUTY_ANGLE;
	conf->tiltback_duty_speed = APPCONF_TNT_TILTBACK_DUTY_SPEED;
	conf->tiltback_duty = APPCONF_TNT_TILTBACK_DUTY;
	conf->is_dutybeep_enabled = APPCONF_TNT_IS_DUTYBEEP_ENABLED;
	conf->tiltback_hv_angle = APPCONF_TNT_TILTBACK_HV_ANGLE;
	conf->tiltback_hv_speed = APPCONF_TNT_TILTBACK_HV_SPEED;
	conf->tiltback_hv = APPCONF_TNT_TILTBACK_HV;
	conf->tiltback_lv_angle = APPCONF_TNT_TILTBACK_LV_ANGLE;
	conf->tiltback_lv_speed = APPCONF_TNT_TILTBACK_LV_SPEED;
	conf->tiltback_lv = APPCONF_TNT_TILTBACK_LV;
	conf->tiltback_ht_angle = APPCONF_TNT_TILTBACK_HT_ANGLE;
	conf->tiltback_ht_speed = APPCONF_TNT_TILTBACK_HT_SPEED;
	conf->tiltback_return_speed = APPCONF_TNT_TILTBACK_RETURN_SPEED;
	conf->tiltback_constant = APPCONF_TNT_TILTBACK_CONSTANT;
	conf->tiltback_constant_erpm = APPCONF_TNT_TILTBACK_CONSTANT_ERPM;
	conf->haptic_buzz_intensity = APPCONF_TNT_HAPTIC_BUZZ_INTENSITY;
	conf->haptic_buzz_min = APPCONF_TNT_HAPTIC_BUZZ_MIN;
	conf->haptic_buzz_current = APPCONF_TNT_HAPTIC_BUZZ_CURRENT;
	conf->haptic_buzz_duty = APPCONF_TNT_HAPTIC_BUZZ_DUTY;
	conf->noseangling_speed = APPCONF_TNT_NOSEANGLING_SPEED;
	conf->inputtilt_remote_type = APPCONF_TNT_INPUTTILT_REMOTE_TYPE;
	conf->inputtilt_angle_limit = APPCONF_TNT_INPUTTILT_ANGLE_LIMIT;
	conf->inputtilt_speed = APPCONF_TNT_INPUTTILT_SPEED;
	conf->inputtilt_smoothing_factor = APPCONF_TNT_INPUTTILT_SMOOTHING_FACTOR;
	conf->inputtilt_invert_throttle = APPCONF_TNT_INPUTTILT_INVERT_THROTTLE;
	conf->inputtilt_deadband = APPCONF_TNT_INPUTTILT_DEADBAND;
	conf->stickytiltval1 = APPCONF_TNT_STICKYTILT_VAL1;
	conf->stickytiltval2 = APPCONF_TNT_STICKYTILT_VAL2;
	conf->stickytilt_holdcurrent = APPCONF_TNT_STICKYTILT_HOLDCURRENT;
	conf->is_stickytilt_enabled = APPCONF_TNT_IS_STICKYTILT_ENABLED;
	conf->startup_pitch_tolerance = APPCONF_TNT_STARTUP_PITCH_TOLERANCE;
	conf->startup_speed = APPCONF_TNT_STARTUP_SPEED;
	conf->startup_simplestart_enabled = APPCONF_SIMPLESTART_ENABLED;
	conf->startup_pushstart_enabled = APPCONF_PUSHSTART_ENABLED;
	conf->startup_dirtylandings_enabled = APPCONF_DIRTYLANDINGS_ENABLED;
	conf->brake_current = APPCONF_TNT_BRAKE_CURRENT;
	conf->overcurrent_margin = APPCONF_TNT_OVERCURRENT_MARGIN;
	conf->overcurrent_period = APPCONF_TNT_OVERCURRENT_PERIOD;
	conf->is_beeper_enabled = APPCONF_TNT_IS_BEEPER_ENABLED;
	conf->is_surgedebug_enabled = APPCONF_TNT_IS_SURGEDEBUG_ENABLED;
	conf->is_tcdebug_enabled = APPCONF_TNT_IS_TCDEBUG_ENABLED;
	conf->is_tunedebug_enabled = APPCONF_TNT_IS_TRIPDEBUG_ENABLED;
	conf->is_yawdebug_enabled = APPCONF_TNT_IS_YAWDEBUG_ENABLED;
	conf->disable_pkg = APPCONF_TNT_DISABLE;
	conf->version = APPCONF_TNT_VERSION;
}

