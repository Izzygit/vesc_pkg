// Copyright 2024 Lukas Hrazky
//
// This file is part of the Refloat VESC package.
//
// Refloat VESC package is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// Refloat VESC package is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.

#include "motor_data_tnt.h"

#include "utils_tnt.h"

#include "vesc_c_if.h"

#include <math.h>

void motor_data_reset(MotorData *m) {
    m->erpm_sign_soft = 0;
    m->acceleration = 0;

    m->erpm_idx = 0;
    for (int i = 0; i < ERPM_ARRAY_SIZE; i++) {
        m->erpm_history[i] = 0;
    }

    biquad_reset(&m->current_biquad);
    biquad_reset(&m->erpm_hs_biquad);
    biquad_reset(&m->erpm_ls_biquad);
}

void motor_data_configure(Biquad *motor_biquad, float frequency) {
    if (frequency > 0) {
        biquad_configure(motor_biquad, BQ_LOWPASS, frequency);
    }
}

void motor_data_update(MotorData *m) {
    m->erpm = VESC_IF->mc_get_rpm();
    m->erpm_hs_filtered = biquad_process(&m->erpm_hs_biquad, m->erpm);
    m->erpm_ls_filtered = biquad_process(&m->erpm_ls_biquad, m->erpm);
    m->abs_erpm = fabsf(m->erpm_hs_filtered);
    m->erpm_sign = sign(m->erpm_hs_filtered);
    
    m->erpm_history[m->erpm_idx] = m->erpm_hs_filtered;
    m->erpm_idx = (m->erpm_idx + 1) % ERPM_ARRAY_SIZE;
    m->last_erpm_idx = m->erpm_idx - ERPM_ARRAY_SIZE; 
    if (m->last_erpm_idx < 0) 
       m->last_erpm_idx += ERPM_ARRAY_SIZE;
	
    m->last_accel_hs = m->accel_hs;
    m->accel_hs =  m->erpm_hs_filtered - m->last_erpm_hs;
    m->last_erpm_hs = m->erpm_hs_filtered;

    m->last_accel_ls = m->accel_ls;
    m->accel_ls =  m->erpm_ls_filtered - m->last_erpm_ls;
    m->last_erpm_ls = m->erpm_ls_filtered;

    m->current = VESC_IF->mc_get_tot_current_directional_filtered();
    m->current_avg = biquad_process(&m->current_biquad, m->current);
    m->braking = m->abs_erpm > 250 && sign(m->current) != m->erpm_sign;

    m->duty_cycle = fabsf(VESC_IF->mc_get_duty_cycle_now());
}
