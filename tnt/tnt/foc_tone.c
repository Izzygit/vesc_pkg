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

#include "foc_tone.h"
#include "utils_tnt.h"

void tone_update(ToneData *tone, RuntimeData *rt, State *state) {
	//This function is updated every code cycle to execute initiated tones
	int index;
	
	if (tone->duration > 30 &&		//Don't allow continuous tones outiside run state
	    state->state != STATE_RUNNING) {	
		end_tone(tone);
	}
	
	if (!tone->pause) { 					//only play or stop tones outside of pause period
		tone->pause_timer = rt->current_time; 		// keep updated until we are in pause state
		if (!tone->tone_in_progress && tone->times != 0) { //times>0 and we are ready for the next tone
			index = min(2, tone->times - 1);	//Use index/times to play frequencies in reverser order: 3 2 1
			if (state->state == STATE_RUNNING) { 	//Choose function based on state
				tone->tone_in_progress = VESC_IF->foc_play_tone(0,  tone->freq[index], tone->voltage);
			} else { tone->tone_in_progress = VESC_IF->foc_beep(tone->freq[index], tone->duration, tone->voltage); }
			tone->timer = rt->current_time;		//Used to track tone duration
			tone->times--; 				//Decrement the times property until 0
		} else if (rt->current_time - tone->timer > tone->duration && tone->tone_in_progress) {
			VESC_IF->foc_stop_audio(true);	//stop foc play tone after duration
			if (tone->times > 0) 		
				tone->pause = true; 		//put in pause if there is another play to do
			tone->tone_in_progress = false; 
		}
	} else if (rt->current_time - tone->pause_timer > 0.1) { //Hard coded pause of 100 ms
		tone->pause = false;
	}
}

void play_tone(ToneData *tone, ToneConfig *toneconfig, RuntimeData *rt, int beep_reason) {
	//This function is used to initiate tones, and only called in specific instances
	if (rt->current_time - tone->timer < toneconfig->delay && 	//This section applies delay to prevent constant repetition
	    tone->beep_reason == beep_reason) 				//if the beep reason remains the same.
		return;			
	
	if (tone->priority < toneconfig->priority) { //Allow for immediate override of higher priority
		tone->tone_in_progress = false;
		VESC_IF->foc_stop_audio(true);
	}
	
	if (!tone->tone_in_progress) {		//Applies tone properties and initiates tone with tone->times > 0
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
	//Used to end continuous tones
	tone->freq[0] = 0;
	tone->freq[1] = 0;
	tone->freq[2] = 0;
	tone->voltage = 0;
	tone->duration = 0;
	tone->times = 0;
	tone->priority = 0;
}

void tone_reset_on_configure(ToneData *tone) {
	//low voltage warnings reset on board start up
	tone->midvolt_activated = false;
	tone->lowvolt_activated = false;
}

void tone_reset(ToneData *tone) {
	tone->duty_tone_count = 0;
	tone->duty_beep_count = 0; 
	tone->midvolt_count = 0;
	tone->lowvolt_count = 0;
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

void tone_configure_all(ToneConfigs *toneconfig, tnt_config *config, ToneData *tone) {
	float beep_voltage = config->is_beeper_enabled ? config->beep_voltage : 0;
	tone_configure(&toneconfig->continuous1, 698, 0, 0, beep_voltage, 601, 1, 0, 1);
	tone_configure(&toneconfig->fastdouble1, 698, 698, 0, beep_voltage, .1, 2, 10, 1);
	tone_configure(&toneconfig->fastdouble2, 880, 880, 0, beep_voltage, .1, 2, 0, 1);
	tone_configure(&toneconfig->slowdouble1, 698, 698, 0, beep_voltage, .3, 2, 30, 1);
	tone_configure(&toneconfig->slowdouble2, 880, 880, 0, beep_voltage, .3, 2, 60, 1);
	tone_configure(&toneconfig->fasttriple1, 698, 698, 698, beep_voltage, .1, 3, 0, 1);
	tone_configure(&toneconfig->slowtriple1, 698, 698, 698, beep_voltage, .3, 3, 10, 4);
	tone_configure(&toneconfig->slowtriple2, 880, 880, 880, beep_voltage, .3, 3, 10, 3);
	tone_configure(&toneconfig->fasttripleup, 880, 784, 698.5, beep_voltage, .1, 3, 10, 2);
	tone_configure(&toneconfig->fasttripledown, 698.5, 784, 880, beep_voltage, .1, 3, 30, 1);
	tone_configure(&toneconfig->slowtripleup, 880, 784, 698.5, beep_voltage, .3, 3, 5, 1);
	tone_configure(&toneconfig->slowtripledown, 698.5, 784, 880, beep_voltage, .3, 3, 5, 4);
	
	beep_voltage = config->is_dutybeep_enabled ? config->beep_voltage : 0;
	tone_configure(&toneconfig->fasttripleupduty, 880, 784, 698.5, beep_voltage, .1, 3, 10, 5);
	
	beep_voltage = config->is_footbeep_enabled ? config->beep_voltage : 0;
	tone_configure(&toneconfig->continuousfootpad, 698, 0, 0, beep_voltage, 601, 1, 0, 2);

	beep_voltage = config->haptic_buzz_duty ? config->tone_volt_high_duty : 0;
	tone_configure(&toneconfig->dutytone, config->tone_freq_high_duty, 0, 0, beep_voltage, 600, 1, 0, 8);

	beep_voltage = config->haptic_buzz_current ? config->tone_volt_high_current : 0;
	tone_configure(&toneconfig->currenttone, config->tone_freq_high_current, 0, 0, beep_voltage, config->overcurrent_period, 1, 0, 6);

	tone->tone_duty = 1.0 * config->tiltback_duty / 100.0; 
	tone->delay_100ms = config->hertz / 10;
	tone->delay_250ms = config->hertz / 4;
	tone->delay_500ms = config->hertz / 2;
	tone->lowvolt_warning = config->lowvolt_warning;
	tone->midvolt_warning = config->midvolt_warning;

	tone_reset_on_configure(tone);
}

void idle_tone(ToneData *tone, ToneConfig *toneconfig, RuntimeData *rt) {
	float input_voltage = VESC_IF->mc_get_input_voltage_filtered();
	if (input_voltage > tone->idle_voltage) {
		// don't beep if the voltage keeps increasing (board is charging)
		if (input_voltage - tone->idle_voltage < .01)
			play_tone(tone, toneconfig, rt, BEEP_CHARGED);
		tone->idle_voltage = input_voltage;
	} else if (rt->current_time - rt->disengage_timer > 2100 &&	// alert user after 35 minutes
	   rt->current_time - rt->disengage_timer < 3000) {		// give up after 50 minutes
		if (rt->current_time - rt->nag_timer > 60) {		// beep every 60 seconds
			rt->nag_timer = rt->current_time;
			play_tone(tone, toneconfig, rt, BEEP_IDLE);
		}
	} else {
		rt->nag_timer = rt->current_time;
		tone->idle_voltage = 0;
	}
}

void temp_recovery_tone(ToneData *tone, ToneConfig *toneconfig, RuntimeData *rt, MotorData *motor) {
	//This function alerts the user once the motor or fets have cooled 10 degrees below the tiltback limit
	if (VESC_IF->mc_temp_motor_filtered() < motor->mc_max_temp_mot - 7 &&
	    tone->motortemp_activated) {
		play_tone(tone, toneconfig, rt, 16);
		tone->motortemp_activated = false;
	} else if (VESC_IF->mc_temp_fet_filtered() < motor->mc_max_temp_fet - 7 &&
	    tone->fettemp_activated) {
		play_tone(tone, toneconfig, rt, 15);
		tone->fettemp_activated = false;
	}
}


void check_tone(ToneData *tone, ToneConfigs *toneconfig, RuntimeData *rt, MotorData *motor) {
	//This function provides a delay before the activation of certain tones
	float input_voltage = VESC_IF->mc_get_input_voltage_filtered();
	
	//Duty FOC Tone and Beep
	if (motor->duty_cycle > tone->tone_duty - .1) { //10% below titltback duty for beep
		if (motor->duty_cycle > tone->tone_duty) { 
			tone->duty_tone_count++; 	//A counter is used to track duty cycle to prevent nuisance trips
			tone->duty_beep_count = 0;
		} else {
			tone->duty_tone_count = 0;	
			tone->duty_beep_count++; 	//A counter is used to track duty cycle to prevent nuisance trips
		}
	} else { 
		tone->duty_tone_count = 0;
		tone->duty_beep_count = 0; 
	}
	
	if (tone->duty_tone_count > tone->delay_100ms) // After we are above duty for 500ms then play tone
		play_tone(tone, &toneconfig->dutytone, rt, TONE_DUTY);
	else if (tone->tone_in_progress && tone->duration == 600) 
		end_tone(tone);	
	else if (tone->duty_beep_count > tone->delay_100ms) // After we are above duty for 500ms then play beep
		play_tone(tone, &toneconfig->fasttripleupduty, rt, BEEP_DUTY);

	//Mid Range Warning
	if (input_voltage < tone->midvolt_warning)
		tone->midvolt_count++; 	//A counter is used to track duty cycle to prevent nuisance trips
	else tone->midvolt_count = 0;

	if (!tone->midvolt_activated && 
	    tone->midvolt_count > tone->delay_500ms) {
		play_tone(tone, &toneconfig->slowtripledown, rt, BEEP_MW);
		tone->midvolt_activated = true;
	}
	
	//Low Range Warning
	if (input_voltage < tone->lowvolt_warning)
		tone->lowvolt_count++; 	//A counter is used to track duty cycle to prevent nuisance trips
	else tone->lowvolt_count = 0;

	if (!tone->lowvolt_activated && 
	    tone->lowvolt_count > tone->delay_500ms) {
		play_tone(tone, &toneconfig->slowtripledown, rt, BEEP_LW);
		tone->lowvolt_activated = true;
	}
}
