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
	int index;
	
	if (tone->duration > 30 &&		//Don't allow continuous tones outiside run state
	    state->state != STATE_RUNNING) {	
		end_tone(&d->tone);
	}
	
	if (!tone->pause) { 					//only play or stop if pause has not been activated
		tone->pause_timer = rt->current_time; 		// keep updated until we are in pause state
		if (!tone->tone_in_progress && tone->times != 0) {
			index = max(3, tone->times - 1);	//Frequencies play in reverser order: 3 2 1
			if (state->state == STATE_RUNNING) { 	//Choose function based on state
				tone->tone_in_progress = VESC_IF->foc_play_tone(0,  tone->freq[index], tone->voltage);
			} else { tone->tone_in_progress = VESC_IF->foc_beep(tone->freq[index], tone->duration, tone->voltage); }
			tone->timer = rt->current_time;
			tone->times--; 				//Decrement the times property until 0
		} else if (rt->current_time - tone->timer > tone->duration && tone->tone_in_progress) {
			if (state->state == STATE_RUNNING)
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
	//Used to play limited duration, repeating, or continuous tones
	if (rt->current_time - tone->timer < toneconfig->delay && 
	    tone->beep_reason == beep_reason) 
		return;			// If we have the same beep reason as the last and we are within the delay period do not update tone->times to prevent beep
	
	if (tone->priority < toneconfig->priority) { //Allow for immediate override of higher priority
		tone->tone_in_progress = false;
		VESC_IF->foc_stop_audio(true);
	}
	
	if (!tone->tone_in_progress) {
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
	tone->midvolt_warning = false;
	tone->lowvolt_warning = false;
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
	tone_configure(&toneconfig->continuous1, 698, 0, 0, 1.5, 601, 1, 0, 2);
	tone_configure(&toneconfig->continuous2, 880, 0, 0, 1.5, 602, 1, 0, 1);
	tone_configure(&toneconfig->fastdouble1, 698, 698, 0, 1.5, .1, 2, 10, 1);
	tone_configure(&toneconfig->fastdouble2, 880, 880, 0, 1.5, .1, 2, 0, 1);
	tone_configure(&toneconfig->slowdouble1, 698, 698, 0, 1.5, .3, 2, 30, 1);
	tone_configure(&toneconfig->slowdouble2, 880, 880, 0, 1.5, .3, 2, 30, 1);
	tone_configure(&toneconfig->fasttriple1, 698, 698, 698, 1.5, .1, 3, 0, 1);
	tone_configure(&toneconfig->fasttriple2, 880, 880, 880, 1.5, .1, 3, 30, 1);
	tone_configure(&toneconfig->slowtriple1, 698, 698, 698, 1.5, .3, 3, 10, 4);
	tone_configure(&toneconfig->slowtriple2, 880, 880, 880, 1.5, .3, 3, 10, 3);
	tone_configure(&toneconfig->fasttripleup,880, 784, 698.5, 1.5, .1, 3, 10, 5);
	tone_configure(&toneconfig->fasttripledown, 698.5, 784, 880, 1.5, .1, 3, 30, 1);
	tone_configure(&toneconfig->slowtripleup, 880, 784, 698.5, 1.5, .3, 3, 5, 1);
	tone_configure(&toneconfig->slowtripledown, 698.5, 784, 880, 1.5, .3, 3, 5, 1);
	tone_configure(&toneconfig->dutytone, config->tone_freq_high_duty, 0, 0, config->tone_volt_high_duty, 600, 1, 0, 8);
	tone_configure(&toneconfig->currenttone, config->tone_freq_high_current, 0, 0, config->tone_volt_high_current, config->overcurrent_period, 1, 0, 6);
}
