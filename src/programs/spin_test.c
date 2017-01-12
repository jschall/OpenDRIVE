/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <stdbool.h>
#include <esc/program.h>

#include <esc/helpers.h>
#include <esc/motor.h>
#include <esc/timing.h>
#include <esc/semihost_debug.h>
#include <esc/drv.h>

void program_init(void) {
    // Calibrate the encoder
//     motor_set_mode(MOTOR_MODE_ENCODER_CALIBRATION);
}

static uint32_t tbegin_us;
static bool started = false;
static float t_max = 0.2f;
static const float f0 = 100.0f;
static const float f1 = 2000.0f;

void program_event_adc_sample(float dt, struct adc_sample_s* adc_sample) {
    uint32_t tnow = micros();
    float t = (tnow-tbegin_us)*1.0e-6f;

    motor_update_state(dt, adc_sample);

    motor_set_iq_ref(1.0f);

//     float beta = t_max / logf(f1/f0);
//     float phi = 2*M_PI_F*beta*f0*(powf(f1/f0, t/t_max)-1.0f);
//
//
//     if (started && t < t_max) {
//         motor_set_iq_ref(3.0f*sin(phi));
//     } else {
//         motor_set_iq_ref(0);
//     }

    if (motor_get_mode() == MOTOR_MODE_DISABLED && !started) {
        motor_set_mode(MOTOR_MODE_FOC_CURRENT);
        started = true;
        tbegin_us = micros();
    }

//     if (motor_get_mode() == MOTOR_MODE_FOC_CURRENT) {
//         uint8_t i;
//         samples[sample_count] = motor_get_iq_est();
//         sample_count++;
//         if (sample_count == 40) {
//             motor_set_mode(MOTOR_MODE_DISABLED);
//             for (i=0; i<40; i++) {
//                 semihost_debug_printf("%d %d\n", (int32_t)i, (int32_t)(samples[i]*1000));
//             }
//         }
//     }

    motor_run_commutation(dt);

    if (started && !drv_get_fault()) {
        motor_print_data(dt);
    }
}
