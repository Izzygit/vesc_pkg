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

#include "pid.h"
#include "utils_tnt.h"
#include <math.h>

float angle_kp_select(float angle, const KpArray *k) {
	float kp_mod = 0;
	float kp_min = 0;
	float scale_angle_min = 0;
	float scale_angle_max = 1;
	float kp_max = 0;
	int i = k->count;
	//Determine the correct kp to use based on angle
	while (i >= 0) {
		if (angle>= k->angle_kp[i][0]) {
			kp_min = k->angle_kp[i][1];
			scale_angle_min = k->angle_kp[i][0];
			if (i == k->count) { //if we are at the highest current only use highest kp
				kp_max = k->angle_kp[i][1];
				scale_angle_max = 90;
			} else {
				kp_max = k->angle_kp[i+1][1];
				scale_angle_max = k->angle_kp[i+1][0];
			}
			i=-1;
		}
		i--;
	}
	
	//Interpolate the kp values according to angle
	kp_mod = lerp(scale_angle_min, scale_angle_max, kp_min, kp_max, angle);
	return kp_mod;
}

void pitch_kp_configure(const tnt_config *config, KpArray *k, int mode){
	float pitch_current[7][2] = { //Accel curve
	{0, 0}, //reserved for kp0 assigned at the end
	{config->pitch1, config->current1},
	{config->pitch2, config->current2},
	{config->pitch3, config->current3},
	{config->pitch4, config->current4},
	{config->pitch5, config->current5},
	{config->pitch6, config->current6},
	};
	float kp0 = config->kp0;
	bool kp_input = config->pitch_kp_input;
	k->kp_rate = config->kp_rate;

	if (mode==2) { //Brake curve
		float temp_pitch_current[7][2] = {
		{0, 0}, //reserved for kp0 assigned at the end
		{config->brakepitch1, config->brakecurrent1},
		{config->brakepitch2, config->brakecurrent2},
		{config->brakepitch3, config->brakecurrent3},
		{config->brakepitch4, config->brakecurrent4},
		{config->brakepitch5, config->brakecurrent5},
		{config->brakepitch6, config->brakecurrent6},
		};
		for (int x = 0; x <= 6; x++) {
			for (int y = 0; y <= 1; y++) {
				pitch_current[x][y] = temp_pitch_current[x][y];
			}
		}
		kp0 = config->brake_kp0;
		k->kp_rate = config->brakekp_rate;
	}

	//Check for current inputs
	int i = 1;
	while (i <= 6){
		if (pitch_current[i][1]!=0 && pitch_current[i][0]>pitch_current[i-1][0]) {
			k->count = i;
			k->angle_kp[i][0]=pitch_current[i][0];
			if (kp_input) {
				k->angle_kp[i][1]=pitch_current[i][1];
			} else {k->angle_kp[i][1]=pitch_current[i][1]/pitch_current[i][0];}
		} else { i=7; }
		i++;
	}
	
	//Check kp0 for an appropriate value, prioritizing kp1
	if (k->angle_kp[1][1] !=0) {
		if (k->angle_kp[1][1] < kp0) {
			k->angle_kp[0][1]= k->angle_kp[1][1]; //If we have a kp1 check to see if it is less than kp0 else reduce kp0
		} else { k->angle_kp[0][1] = kp0; } //If less than kp1 it is OK
	} else if (kp0 == 0) { //If no currents and no kp0
		k->angle_kp[0][1] = 5; //default 5
	} else { k->angle_kp[0][1] = kp0; }//passes all checks, it is ok 
}

void angle_kp_reset(KpArray *k) {
	//necessary only for the pitch kparray
	for (int x = 0; x <= 6; x++) {
		for (int y = 0; y <= 1; y++) {
			k->angle_kp[x][y] = 0;
		}
	}
	k->count = 0;
}

void roll_kp_configure(const tnt_config *config, KpArray *k, int mode){
	float accel_roll_kp[7][2] = { //Accel curve
	{0, 0}, 
	{config->roll1, config->roll_kp1},
	{config->roll2, config->roll_kp2},
	{config->roll3, config->roll_kp3},
	{0, 0},
	{0, 0},
	{0, 0},
	};
	
	float brake_roll_kp[7][2] = { //Brake Curve
	{0, 0}, 
	{config->brkroll1, config->brkroll_kp1},
	{config->brkroll2, config->brkroll_kp2},
	{config->brkroll3, config->brkroll_kp3},
	{0, 0},
	{0, 0},
	{0, 0},
	};	

	for (int x = 0; x <= 6; x++) {
		for (int y = 0; y <= 1; y++) {
			k->angle_kp[x][y] = (mode==2) ? brake_roll_kp[x][y] : accel_roll_kp[x][y];
		}
	}
	
	if (k->angle_kp[1][1]<k->angle_kp[2][1] && k->angle_kp[1][0]<k->angle_kp[2][0]) {
		if (k->angle_kp[2][1]<k->angle_kp[3][1] && k->angle_kp[2][0]<k->angle_kp[3][0]) {
			k->count = 3;
		} else {k->count = 2;}
	} else if (k->angle_kp[1][1] >0 && k->angle_kp[1][0]>0) {
		k->count = 1;
	} else {k->count = 0;}
}

void yaw_kp_configure(const tnt_config *config, KpArray *k, int mode){
	float accel_yaw_kp[7][2] = { //Accel curve
	{0, 0}, 
	{config->yaw1 / config->hertz, config->yaw_kp1},
	{config->yaw2 / config->hertz, config->yaw_kp2},
	{config->yaw3 / config->hertz, config->yaw_kp3},
	{0, 0},
	{0, 0},
	{0, 0},
	};
	
	float brake_yaw_kp[7][2] = { //Brake Curve
	{0, 0}, 
	{config->brkyaw1 / config->hertz, config->brkyaw_kp1},
	{config->brkyaw2 / config->hertz, config->brkyaw_kp2},
	{config->brkyaw3 / config->hertz, config->brkyaw_kp3},
	{0, 0},
	{0, 0},
	{0, 0},
	};	

	for (int x = 0; x <= 6; x++) {
		for (int y = 0; y <= 1; y++) {
			k->angle_kp[x][y] = (mode==2) ? brake_yaw_kp[x][y] : accel_yaw_kp[x][y];
		}
	}
	
	if (k->angle_kp[1][1]<k->angle_kp[2][1] && k->angle_kp[1][0]<k->angle_kp[2][0]) {
		if (k->angle_kp[2][1]<k->angle_kp[3][1] && k->angle_kp[2][0]<k->angle_kp[3][0]) {
			k->count = 3;
		} else {k->count = 2;}
	} else if (k->angle_kp[1][1] >0 && k->angle_kp[1][0]>0) {
		k->count = 1;
	} else {k->count = 0;}
}

float erpm_scale(float lowvalue, float highvalue, float lowscale, float highscale, float abs_erpm){ 
	float scaler = lerp(lowvalue, highvalue, lowscale, highscale, abs_erpm);
	if (lowscale < highscale) {
		scaler = min(max(scaler, lowscale), highscale);
	} else { scaler = max(min(scaler, lowscale), highscale); }
	return scaler;
}

void apply_stability(PidData *p, MotorData *m, RemoteData *remote, tnt_config *config) {
	float speed_stabl_mod = 0;
	float throttle_stabl_mod = 0;	
	float stabl_mod = 0;
	if (config->enable_throttle_stability) {
		throttle_stabl_mod = fabsf(remote->inputtilt_interpolated) / config->inputtilt_angle_limit; 	//using inputtilt_interpolated allows the use of sticky tilt and inputtilt smoothing
	}
	if (config->enable_speed_stability && m->abs_erpm > 1.0 * config->stabl_min_erpm) {		
		speed_stabl_mod = fminf(1 ,										// Do not exceed the max value.				
				lerp(config->stabl_min_erpm, config->stabl_max_erpm, 0, 1, m->abs_erpm));
	}
	stabl_mod = fmaxf(speed_stabl_mod,throttle_stabl_mod);
	float step_size = stabl_mod > p->stabl ? p->stabl_step_size_up : p->stabl_step_size_down;
	rate_limitf(&p->stabl, stabl_mod, step_size); 
	p->stability_kp = 1 + p->stabl * config->stabl_pitch_max_scale / 100; //apply dynamic stability for pitch kp
	p->stability_kprate = 1 + p->stabl * config->stabl_rate_max_scale / 100;
}

void check_brake_kp(PidData *p, State *state, tnt_config *config, KpArray *roll_brake_kp, KpArray *yaw_brake_kp) {
	p->brake_roll = roll_brake_kp->count!=0 && state->braking_pos;
	p->brake_pitch = config->brake_curve && state->braking_pos;
	p->brake_yaw = yaw_brake_kp->count!=0 && state->braking_pos;
}

float roll_erpm_scale(PidData *p, State *state, MotorData *m, KpArray *roll_accel_kp, tnt_config *config) {
	//Apply ERPM Scale
	float erpmscale = 1;
	if ((p->brake_roll && m->abs_erpm < 750) ||
		state->sat == SAT_CENTERING) { 				
		// If we want to actually stop at low speed reduce kp to 0
		erpmscale = 0;
	} else if (roll_accel_kp->count!=0 && m->abs_erpm < config->rollkp_higherpm) { 
		erpmscale = 1 + erpm_scale(config->rollkp_lowerpm, config->rollkp_higherpm, config->rollkp_maxscale / 100.0, 0, m->abs_erpm);
	} else if (roll_accel_kp->count!=0 && m->abs_erpm > config->roll_hs_lowerpm) { 
		erpmscale = 1 + erpm_scale(config->roll_hs_lowerpm, config->roll_hs_higherpm, 0, config->roll_hs_maxscale / 100.0, m->abs_erpm);
	}
	return erpmscale;
}

void reset_pid(PidData *p) {
	p->pid_value = 0;
	p->pid_mod = 0;
	p->roll_pid_mod = 0;
	p->yaw_pid_mod = 0;
	p->stabl = 0;
	p->prop_smooth = 0;
	p->abs_prop_smooth = 0;
	p->softstart_pid_limit = 0;
}

void apply_soft_start(PidData *p, MotorData *m) {
	if (p->softstart_pid_limit < m->mc_current_max) {
		p->pid_mod = fminf(fabsf(p->pid_mod), p->softstart_pid_limit) * sign(p->pid_mod);
		p->softstart_pid_limit += p->softstart_step_size;
	}
}

void configure_pid(PidData *p, tnt_config *config) {
	//Dynamic Stability
	p->stabl_step_size_up = 1.0 * config->stabl_ramp / 100.0 / config->hertz;
	p->stabl_step_size_down = 1.0 * config->stabl_ramp_down / 100.0 / config->hertz;
	
	// Feature: Soft Start
	p->softstart_step_size = 100.0 / config->hertz;
}

void tone_update(ToneData *tone, RuntimeData *rt, State *state) {
	int index;
	if (!tone->pause) { //only play or stop if pause has not been activated
		tone->pause_timer = rt->current_time; // keep updated until we are in pause state
		if (!tone->tone_in_progress && tone->times != 0) {
			index = tone->times - 1;
			if (state->state == STATE_RUNNING) {
				tone->tone_in_progress = VESC_IF->foc_play_tone(0,  tone->freq[index], tone->voltage);
			} else { tone->tone_in_progress = VESC_IF->foc_beep(tome->freq[index], tone->duration, tone->voltage);
			tone->timer = rt->current_time;
		} else if (rt->current_time - tone->timer > tone->duration && tone->tone_in_progress) {
			if (state->state == STATE_RUNNING)
				VESC_IF->foc_stop_audio(true);
			tone->times--; 
			if (tone->times > 0) {
				tone->pause = true; //put in pause if there is another play to do
			} else { tone->tone_in_progress = false; }
		}
	else if (rt->current_time - tone->pause_timer > 0.1) {
		tone->pause = false;
}

void play_tone(ToneData *tone, ToneConfig *toneconfig, RuntimeData *rt, int beep_reason) {
	//Used to play limited duration, repeating, or continuous tones
	if (rt->current_time - tone->timer < tonecofig->delay && 
	    tone->beep_reason == beep_reason) 
		return;			// If we have the same beep reason as the last and we are within the delay period do not update tone->times to prevent beep
	
	if (!tone->tone_in_progress || tone->priority < priority) {
		tone->freq[0] = toneconfig->freq[0];
		tone->freq[1] = toneconfig->freq[1];
		tone->freq[2] = toneconfig->freq[2];
		tone->voltage = toneconfig->voltage;
		tone->duration = toneconfig->duration;
		tone->priority = toneconfig->priority;
		tone->times = toneconfig->times;
		tone->pause = false;
		tone->beep_reason = beep_reason;
	}
}

void end_tone(ToneData *tone) {
	//Used to end continous tones
	tone->freq[0] = 0;
	tone->freq[1] = 0;
	tone->freq[2] = 0;
	tone->voltage = 0;
	tone->duration = 0;
	tone->times = 0;
	tone->priority = 0;
}

void tone_reset(ToneData *tone) {
	tone->tone_in_progress = false;
	end_tone(tone);
}

void tone_configure(ToneConfig *toneconfig, float freq1, float freq2, float freq3, float voltage, float duration, int times, float delay, int priority) {
	toneconfig->freq[0] = freq1;
	toneconfig->freq[1] = freq2;
	toneconfig->freq[2] = freq3;
	toneconfig->voltage = voltage;
	toneconfig->duration = duration;
	toneconfig->times = times;
	toneconfig->delay = delay;
	toneconfig->priority = priority;
}

void tone_configure_all(ToneConfigs *toneconfig, tnt_config *config) {
	tone_configure(&toneconfig->continuous1, 800, 0, 0, 2, 601, 1, 0, 1);
	tone_configure(&toneconfig->continuous2, 1000, 0, 0, 2, 602, 1, 0, 1);
	tone_configure(&toneconfig->fastdouble1, 800, 800, 0, 2, .2, 2, 30, 1);
	tone_configure(&toneconfig->fastdouble2, 1000, 1000, 0, 2, .2, 2, 30, 1);
	tone_configure(&toneconfig->slowdouble1, 800, 800, 0, 2, .5, 2, 30, 1);
	tone_configure(&toneconfig->slowdouble2, 1000, 1000, 0, 2, .5, 2, 30, 1);
	tone_configure(&toneconfig->fasttriple1, 800, 800, 800, 2, .2, 3, 30, 1);
	tone_configure(&toneconfig->fasttriple2, 1000, 1000, 1000, 2, .2, 3, 30, 1);
	tone_configure(&toneconfig->slowtriple1, 800, 800, 800, 2, .5, 3, 30, 1);
	tone_configure(&toneconfig->slowtriple2, 1000, 1000, 1000, 2, .5, 3, 30, 1);
	tone_configure(&toneconfig->fasttripleup,700, 800, 1000, 2, .2, 3, 30, 1);
	tone_configure(&toneconfig->fasttripledown, 1000, 800, 700, 2, .2, 3, 30, 1);
	tone_configure(&toneconfig->slowtripleup, 700, 800, 1000, 2, .5, 3, 30, 1);
	tone_configure(&toneconfig->slowtripledown, 1000, 800, 700, 2, .5, 3, 30, 1);
	tone_configure(&toneconfig->dutytone, config->tone_freq_high_duty, 0, 0, config->tone_volt_high_duty, 600, 1, 0, 8);
	tone_configure(&toneconfig->currenttone, config->tone_freq_high_current, 0, 0, config->tone_volt_high_current, config->overcurrent_period, 1, 0, 6);
}
