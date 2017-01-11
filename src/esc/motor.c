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

#include <esc/motor.h>

#include <string.h>
#include <esc/helpers.h>
#include <esc/pwm.h>
#include <esc/adc.h>
#include <esc/drv.h>
#include <esc/encoder.h>
#include <esc/timing.h>
#include <esc/curr_pid.h>
#include <esc/serial.h>
#include <esc/semihost_debug.h>
#include <esc/slip.h>

#include <esc/can.h>

// config things - to be made params later
static uint8_t mot_n_poles = 7;
static const float mot_Kt = 30.0f/(M_PI_F*360.0f);
static const float mot_R = 0.11f;
static const float mot_L = 30.0f * 1e-6f;
static const float mot_rotor_inertia = 0.0002f;
static const float i_meas_var = SQ(0.03f);
static const float load_torque_noise = 1.11426f;
static const float v_noise = 0.01f;
static const float load_torque = 0.0f;

static float elec_theta_bias = 0.0f;
static bool reverse = false;
static const float curr_KR = 0.0f;
static const float curr_KP = .15f;
static const float curr_KI = 500.0f;
static const float vsense_div = 20.0f;
static const float csa_G = 10.0f;
static const float csa_R = 0.001f;
// it takes approximately 50 timer clock cycles to sample the current sensors. PWM period is 2000 timer clock cycles
// TODO reconstruct current measurement to allow max_duty=1.0
static const float max_duty = 1.0f;
static const float calibration_voltage = 2.0f;

static float csa_cal[3] = {0.0f, 0.0f, 0.0f}; // current sense amplifier calibration
static float vbatt_m = 0.0f; // battery voltage
static float ia_m = 0.0f, ib_m = 0.0f, ic_m = 0.0f; // phase currents
static float ialpha_m = 0.0f, ibeta_m = 0.0f, igamma_m = 0.0f; // alpha-beta-gamma (clarke) transform of phase currents
static float id_meas = 0.0f, iq_meas = 0.0f; // dqo transform of phase currents
static float mech_theta_m = 0.0f; // mechanical rotor angle
static float prev_mech_theta_m = 0.0f; // previous mechanical rotor angle for differentiation
static float elec_theta_m = 0.0f; // electrical rotor angle
static float mech_omega_est = 0.0f; // mechanical rotor angular velocity
static float alpha_out = 0.0f, beta_out = 0.0f;
static enum motor_mode_t motor_mode = MOTOR_MODE_DISABLED;

struct {
    uint32_t start_time_us;
    uint8_t step;
    float mech_theta_0;
} encoder_calibration_state;

static struct curr_pid_param_s iq_pid_param;
static struct curr_pid_state_s iq_pid_state;

static struct curr_pid_param_s id_pid_param;
static struct curr_pid_state_s id_pid_state;

static void retrieve_adc_measurements(void);
static void retrieve_encoder_measurement(void);
static void update_estimates(float dt);
static void load_pid_configs(void);
static void transform_a_b_c_to_alpha_beta_gamma(float a, float b, float c, float* alpha, float* beta, float* gamma);
static void transform_d_q_to_alpha_beta(float d, float q, float* alpha, float* beta);
static void transform_alpha_beta_to_d_q(float alpha, float beta, float* d, float* q);
static void svgen(float alpha, float beta, float* a, float* b, float* c);

static bool ekf_running = false;
static float state[4];
static float cov[10];

static uint32_t ekf_update_count;

static void ekf_init(float theta)
{
    ekf_running = true;
    ekf_update_count = 0;
    state[0] = 0.0f; // omega
    state[1] = theta;
    state[2] = id_meas;
    state[3] = iq_meas;
    // 0 1 2 3
    //   4 5 6
    //     7 8
    //       9
    cov[0] = 10.0f;
    cov[1] = 0.0f;
    cov[2] = 0.0f;
    cov[3] = 0.0f;
    cov[4] = SQ(M_PI_F);
    cov[5] = 0.0f;
    cov[6] = 0.0f;
    cov[7] = i_meas_var;
    cov[8] = 0.0f;
    cov[9] = i_meas_var;
}

static void ekf_update(float dt)
{
//     float subx[67];
//     float state_n[4];
//     float cov_n[10];
//
//     subx[0] = 1.0/mot_rotor_inertia;
//     subx[1] = id_meas - state[2];
//     subx[2] = ((mot_L)*(mot_L));
//     subx[3] = dt*mot_R - mot_L;
//     subx[4] = cov[7]*subx[3];
//     subx[5] = cov[8]*state[0];
//     subx[6] = dt*mot_L*mot_n_poles;
//     subx[7] = subx[6]*(cov[2]*state[3] + subx[5]);
//     subx[8] = cov[8]*subx[3];
//     subx[9] = subx[6]*(cov[3]*state[3] + cov[9]*state[0]);
//     subx[10] = cov[2]*subx[3];
//     subx[11] = subx[6]*(cov[0]*state[3] + cov[3]*state[0]);
//     subx[12] = i_meas_var*subx[2];
//     subx[13] = ((dt)*(dt))*((v_noise)*(v_noise));
//     subx[14] = subx[12] + subx[13] - subx[3]*(-subx[4] + subx[7]) + subx[6]*(state[0]*(-subx[8] + subx[9]) + state[3]*(-subx[10] + subx[11]));
//     subx[15] = 1.0/subx[14];
//     subx[16] = 1.0/mot_L;
//     subx[17] = -dt*mot_R*subx[16] + 1;
//     subx[18] = mot_n_poles*state[2];
//     subx[19] = 0.666666666666667*mot_Kt;
//     subx[20] = dt*(-subx[16]*subx[19] - subx[18]);
//     subx[21] = dt*mot_n_poles;
//     subx[22] = state[0]*subx[21];
//     subx[23] = cov[2]*subx[20] - cov[7]*subx[22] + cov[8]*subx[17];
//     subx[24] = cov[3]*subx[20] + cov[9]*subx[17] - subx[21]*subx[5];
//     subx[25] = cov[0]*subx[20] - cov[2]*subx[22] + cov[3]*subx[17];
//     subx[26] = state[3]*subx[21]*subx[25] + subx[17]*subx[23] + subx[22]*subx[24];
//     subx[27] = dt*(mot_L*subx[18] + subx[19]);
//     subx[28] = cov[3]*subx[27] + cov[9]*subx[3] + subx[5]*subx[6];
//     subx[29] = cov[0]*subx[27] + cov[2]*mot_L*subx[22] + cov[3]*subx[3];
//     subx[30] = subx[12] + subx[13] + subx[3]*(subx[4] - subx[7]) - subx[6]*(state[0]*(subx[8] - subx[9]) + state[3]*(subx[10] - subx[11]));
//     subx[31] = cov[2]*subx[27] + cov[7]*mot_L*subx[22] + subx[8];
//     subx[32] = dt*mot_L*mot_n_poles*state[0];
//     subx[33] = 1.0/(subx[12]*subx[30] + subx[30]*subx[31]*subx[32] + subx[30]*(subx[13] + subx[27]*subx[29] + subx[28]*subx[3]) - (-subx[31]*subx[3] + subx[6]*(state[0]*subx[28] + state[3]*subx[29]))*(subx[27]*(-subx[10] + subx[11]) + subx[32]*(-subx[4] + subx[7]) + subx[3]*(-subx[8] + subx[9])));
//     subx[34] = cov[3]*state[3]*subx[21] + cov[8]*subx[17] + cov[9]*state[0]*subx[21];
//     subx[35] = cov[0]*state[3]*subx[21] + cov[2]*subx[17] + cov[3]*state[0]*subx[21];
//     subx[36] = cov[2]*state[3]*subx[21] + cov[7]*subx[17] + subx[21]*subx[5];
//     subx[37] = subx[17]*subx[34] + subx[20]*subx[35] - subx[22]*subx[36];
//     subx[38] = powf(mot_L, 6)*subx[26]*subx[30]*subx[33]*subx[37]/((subx[14])*(subx[14])) + subx[15]*subx[2];
//     subx[39] = dt*mot_Kt*subx[0];
//     subx[40] = cov[2] + cov[8]*subx[39];
//     subx[41] = cov[3] + cov[9]*subx[39];
//     subx[42] = cov[0] + cov[3]*subx[39];
//     subx[43] = state[3]*subx[21]*subx[42] + subx[17]*subx[40] + subx[22]*subx[41];
//     subx[44] = subx[17]*subx[41] + subx[20]*subx[42] - subx[22]*subx[40];
//     subx[45] = ((mot_L)*(mot_L)*(mot_L)*(mot_L))*subx[15]*subx[26]*subx[30]*subx[33];
//     subx[46] = iq_meas - state[3];
//     subx[47] = subx[2]*subx[30]*subx[33];
//     subx[48] = ((mot_L)*(mot_L)*(mot_L)*(mot_L))*subx[15]*subx[30]*subx[33]*subx[37];
//     subx[49] = cov[2]*subx[21] + cov[5];
//     subx[50] = cov[3]*subx[21] + cov[6];
//     subx[51] = cov[0]*subx[21] + cov[1];
//     subx[52] = state[3]*subx[21]*subx[51] + subx[17]*subx[49] + subx[22]*subx[50];
//     subx[53] = subx[17]*subx[50] + subx[20]*subx[51] - subx[22]*subx[49];
//     subx[54] = state[3]*subx[21]*subx[35] + subx[13]/subx[2] + subx[17]*subx[36] + subx[22]*subx[34];
//     subx[55] = subx[38]*subx[54];
//     subx[56] = subx[26]*subx[48];
//     subx[57] = subx[37]*subx[47];
//     subx[58] = subx[48]*subx[54];
//     subx[59] = subx[13]/subx[2] + subx[17]*subx[24] + subx[20]*subx[25] - subx[22]*subx[23];
//     subx[60] = -subx[38]*subx[43] + subx[44]*subx[45];
//     subx[61] = subx[43]*subx[48] - subx[44]*subx[47];
//     subx[62] = cov[1]*state[3]*subx[21] + cov[5]*subx[17] + cov[6]*subx[22] + subx[21]*subx[35];
//     subx[63] = cov[1]*subx[20] - cov[5]*subx[22] + cov[6]*subx[17] + subx[21]*subx[25];
//     subx[64] = -subx[38]*subx[52] + subx[45]*subx[53];
//     subx[65] = -subx[47]*subx[53] + subx[48]*subx[52];
//     subx[66] = subx[56] + 1;
//     state_n[0] = dt*(-load_torque*subx[0] + mot_Kt*state[3]*subx[0]) + state[0] + subx[1]*(subx[38]*subx[43] - subx[44]*subx[45]) + subx[46]*(-subx[43]*subx[48] + subx[44]*subx[47]);
//     state_n[1] = state[1] + subx[1]*(subx[38]*subx[52] - subx[45]*subx[53]) + subx[22] + subx[46]*(subx[47]*subx[53] - subx[48]*subx[52]);
//     state_n[2] = dt*(id_pid_state.output*subx[16] - mot_R*state[2]*subx[16] + mot_n_poles*state[0]*state[3]) + state[2] + subx[1]*(subx[55] - subx[56]) + subx[46]*(subx[57] - subx[58]);
//     state_n[3] = dt*(iq_pid_state.output*subx[16] - mot_R*state[3]*subx[16] - state[0]*subx[16]*subx[19] - state[0]*subx[18]) + state[3] + subx[1]*(subx[26]*subx[38] - subx[45]*subx[59]) + subx[46]*(subx[47]*subx[59] - subx[56]);
//     cov_n[0] = ((dt)*(dt))*((load_torque_noise)*(load_torque_noise))/((mot_rotor_inertia)*(mot_rotor_inertia)) + subx[39]*subx[41] + subx[42] + subx[60]*(subx[34]*subx[39] + subx[35]) + subx[61]*(subx[24]*subx[39] + subx[25]);
//     cov_n[1] = cov[1] + cov[6]*subx[39] + subx[21]*subx[42] + subx[60]*subx[62] + subx[61]*subx[63];
//     cov_n[2] = subx[26]*subx[61] + subx[43] + subx[54]*subx[60];
//     cov_n[3] = subx[37]*subx[60] + subx[44] + subx[59]*subx[61];
//     cov_n[4] = cov[1]*subx[21] + cov[4] + subx[21]*subx[51] + subx[62]*subx[64] + subx[63]*subx[65];
//     cov_n[5] = subx[26]*subx[65] + subx[52] + subx[54]*subx[64];
//     cov_n[6] = subx[37]*subx[64] + subx[53] + subx[59]*subx[65];
//     cov_n[7] = subx[26]*(-subx[57] + subx[58]) + subx[54]*(-subx[55] + subx[66]);
//     cov_n[8] = subx[37]*(-subx[55] + subx[66]) + subx[59]*(-subx[57] + subx[58]);
//     cov_n[9] = subx[37]*(-subx[26]*subx[38] + subx[45]*subx[59]) + subx[59]*(-subx[47]*subx[59] + subx[66]);
//
//     state_n[1] = fmodf(state_n[1], 2.0f*M_PI_F);
//
//     if (ekf_running) {
//         uint8_t i;
//
//         ekf_update_count++;
//
//         for (i=0; i<4; i++) {
//             if (isnan(state_n[i]) || isinf(state_n[i])) {
//                 ekf_running = false;
//             }
//         }
//         for (i=0; i<10; i++) {
//             if (isnan(cov_n[i]) || isinf(cov_n[i])) {
//                 ekf_running = false;
//             }
//         }
//
//         if (!ekf_running) {
//             for (i=0; i<67; i++) {
//                 semihost_debug_printf("subx[%u] %u %u\n", i, isnan(subx[i]), isinf(subx[i]));
//             }
//             for (i=0; i<4; i++) {
//                 semihost_debug_printf("state_n[%u] %u %u\n", i, isnan(state_n[i]), isinf(state_n[i]));
//             }
//             for (i=0; i<10; i++) {
//                 semihost_debug_printf("cov_n[%u] %u %u\n", i, isnan(cov_n[i]), isinf(cov_n[i]));
//             }
//         }
//
//         if (ekf_update_count > 500) {
//             ekf_running = false;
//         }
//
//         memcpy(state, state_n, sizeof(state));
//         memcpy(cov, cov_n, sizeof(cov));
//
//
//         struct canbus_msg msg;
//         msg.id = 50+msg_idx;
//         msg.ide = false;
//         msg.rtr = false;
//         msg.dlc = 8;
//
//         switch (msg_idx) {
//             case 0: {
//                 memcpy(&msg.data[0], &state[0], sizeof(float));
//                 memcpy(&msg.data[4], &mech_omega_est, sizeof(float));
//                 break;
//             }
//             case 1:
//                 memcpy(&msg.data[0], &state[1], sizeof(float));
//                 memcpy(&msg.data[4], &elec_theta_m, sizeof(float));
//                 break;
//             case 2:
//                 memcpy(&msg.data[0], &state[2], sizeof(float));
//                 memcpy(&msg.data[4], &id_meas, sizeof(float));
//                 break;
//             case 3:
//                 memcpy(&msg.data[0], &state[3], sizeof(float));
//                 memcpy(&msg.data[4], &iq_meas, sizeof(float));
//                 break;
//         };
//         canbus_send_message(&msg);
//
//         elec_theta_m = state[1];
//
//         msg_idx = (msg_idx+1)%4;
//     }


}

void motor_print_data(float dt) {
    uint8_t slip_msg[64];
    uint8_t slip_msg_len = 0;
    uint8_t i;
    uint32_t tnow_us = micros();
    float omega_e = mech_omega_est*mot_n_poles;

    for (i=0; i<sizeof(uint32_t); i++) {
        slip_encode_and_append(((uint8_t*)&tnow_us)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&dt)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&elec_theta_m)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&omega_e)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&ialpha_m)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&ibeta_m)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&alpha_out)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&beta_out)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    slip_msg[slip_msg_len++] = SLIP_END;

    serial_send_dma(slip_msg_len, slip_msg);
}

void motor_init(void)
{
    // calibrate phase currents
    uint16_t i;
    drv_csa_cal_mode_on();
    usleep(50);
    csa_cal[0] = 0;
    csa_cal[1] = 0;
    csa_cal[2] = 0;
    for(i=0; i<1000; i++) {
        float csa_v_a, csa_v_b, csa_v_c;
        adc_wait_for_sample();
        adc_get_csa_v(&csa_v_a, &csa_v_b, &csa_v_c);
        csa_cal[0] += csa_v_a;
        csa_cal[1] += csa_v_b;
        csa_cal[2] += csa_v_c;
    }
    drv_csa_cal_mode_off();
    csa_cal[0] /= 1000;
    csa_cal[1] /= 1000;
    csa_cal[2] /= 1000;

    // initialize encoder filter states
    retrieve_encoder_measurement();
    prev_mech_theta_m = mech_theta_m;
    mech_omega_est = 0.0f;
}

void motor_update_state(float dt)
{
    retrieve_adc_measurements();
    retrieve_encoder_measurement();
    update_estimates(dt);
}

void motor_run_commutation(float dt)
{
    load_pid_configs();

    id_pid_param.dt = iq_pid_param.dt  = dt;

    id_pid_param.i_meas = id_meas;
    iq_pid_param.i_meas = iq_meas;

    float alpha, beta, a, b, c;
    switch (motor_mode) {
        case MOTOR_MODE_DISABLED:
            pwm_set_phase_duty(0.0f, 0.0f, 0.0f);
            break;

        case MOTOR_MODE_FOC_CURRENT:
            id_pid_param.i_ref = 0.0f;
            id_pid_param.output_limit = max_duty*vbatt_m;
            curr_pid_run(&id_pid_param, &id_pid_state);

            // limit iq such that
            iq_pid_param.output_limit = sqrtf(MAX(SQ(id_pid_param.output_limit)-SQ(id_pid_state.output),0.0f));
            curr_pid_run(&iq_pid_param, &iq_pid_state);

            transform_d_q_to_alpha_beta(id_pid_state.output, iq_pid_state.output, &alpha_out, &beta_out);

            svgen(alpha_out/vbatt_m, beta_out/vbatt_m, &a, &b, &c);

            if (!reverse) {
                pwm_set_phase_duty(a, b, c);
            } else {
                pwm_set_phase_duty(a, c, b);
            }
            break;

        case MOTOR_MODE_ENCODER_CALIBRATION: {
            float t = (micros() - encoder_calibration_state.start_time_us)*1.0e-6f;
            float theta = 0.0f;
            float v = constrain_float(calibration_voltage/vbatt_m, 0.0f, max_duty);

            switch(encoder_calibration_state.step) {
                case 0:
                    // the motor is given 1 second to settle
                    theta = 0.0f;
                    if (t > 1.0f) {
                        encoder_calibration_state.mech_theta_0 = mech_theta_m;
                        encoder_calibration_state.step = 1;
                    }

                    break;
                case 1:
                    // theta rotates to 180deg at the 2.0 second mark and is given 0.5 seconds to settle
                    theta = constrain_float(M_PI_F * (t-1.0f)/1.0f, 0.0f, M_PI_F);
                    if (t > 2.5f) {
                        // mot_n_poles = delta_elec_angle/delta_mech_angle
                        float angle_diff = wrap_pi(mech_theta_m - encoder_calibration_state.mech_theta_0);
                        mot_n_poles = (uint8_t)roundf((M_PI_F)/fabsf(angle_diff));

                        // rotating the field in the positive direction should have rotated the encoder in the positive direction too
                        reverse = angle_diff < 0;

                        encoder_calibration_state.step = 2;
                    }

                    break;
                case 2:
                    theta = M_PI_F;

                    // correct elec_theta_bias to zero atan2f(iq_meas, id_meas), which represents the electrical angle error
                    elec_theta_bias = wrap_pi(elec_theta_bias - atan2f(iq_meas, id_meas));

                    motor_set_mode(MOTOR_MODE_DISABLED);
                    semihost_debug_printf("mot_n_poles %u\n", mot_n_poles);
                    break;
            }

            alpha = v * cosf(theta);
            beta = v * sinf(theta);

            svgen(alpha, beta, &a, &b, &c);

            pwm_set_phase_duty(a, b, c);

            break;
        }

        case MOTOR_MODE_PHASE_VOLTAGE_TEST: {
            float theta = wrap_2pi(millis()*1e-3f);
            float v = constrain_float(calibration_voltage/vbatt_m, 0.0f, max_duty);

            alpha = v * cosf(theta);
            beta = v * sinf(theta);

            svgen(alpha, beta, &a, &b, &c);

            pwm_set_phase_duty(a, b, c);
            break;
        }
    }
}

void motor_set_mode(enum motor_mode_t mode)
{
    if (motor_mode == mode) {
        return;
    }

    motor_mode = mode;

    if (motor_mode == MOTOR_MODE_DISABLED) {
        drv_6_pwm_mode();
        pwm_set_phase_duty(0.0f, 0.0f, 0.0f);
    } else {
        drv_3_pwm_mode();
    }

    if (motor_mode == MOTOR_MODE_ENCODER_CALIBRATION) {
        encoder_calibration_state.start_time_us = micros();
        encoder_calibration_state.step = 0;
    }

    // reset PID states and inputs
    memset(&id_pid_state,0,sizeof(id_pid_state));
    memset(&iq_pid_state,0,sizeof(iq_pid_state));
    id_pid_param.i_ref = 0.0f;
    iq_pid_param.i_ref = 0.0f;

    if (motor_mode == MOTOR_MODE_FOC_CURRENT) {
        ekf_init(elec_theta_m);
    }
}

void motor_set_iq_ref(float iq_ref)
{
    iq_pid_param.i_ref = iq_ref;
}

enum motor_mode_t motor_get_mode(void)
{
    return motor_mode;
}

float motor_get_phys_rotor_angle(void)
{
    return mech_theta_m;
}

float motor_get_phys_rotor_ang_vel(void)
{
    return mech_omega_est;
}

float motor_get_elec_rotor_angle(void)
{
    return mech_theta_m;
}

float motor_get_vbatt(void)
{
    return vbatt_m;
}

float motor_get_iq_meas(void)
{
    return iq_meas;
}

static void retrieve_adc_measurements(void)
{
    // retrieve battery measurement
    vbatt_m = adc_get_vsense_v()*vsense_div;

    // retrieve current sense amplifier measurement
    float csa_v_0, csa_v_1, csa_v_2;
    adc_get_csa_v(&csa_v_0, &csa_v_1, &csa_v_2);
    ia_m = (csa_v_0-csa_cal[0])/(csa_G*csa_R);
    ib_m = (csa_v_1-csa_cal[1])/(csa_G*csa_R);
    ic_m = (csa_v_2-csa_cal[2])/(csa_G*csa_R);

    float duty_a, duty_b, duty_c;
    pwm_get_phase_duty(&duty_a, &duty_b, &duty_c);

    if (duty_a > duty_b && duty_a > duty_c) {
        ia_m = -ib_m-ic_m;
    } else if (duty_b > duty_a && duty_b > duty_c) {
        ib_m = -ia_m-ic_m;
    } else {
        ic_m = -ia_m-ib_m;
    }

    if (!reverse) {
        ib_m = (csa_v_1-csa_cal[1])/(csa_G*csa_R);
        ic_m = (csa_v_2-csa_cal[2])/(csa_G*csa_R);
    } else {
        ib_m = (csa_v_2-csa_cal[2])/(csa_G*csa_R);
        ic_m = (csa_v_1-csa_cal[1])/(csa_G*csa_R);
    }
}

static void retrieve_encoder_measurement(void)
{
    mech_theta_m = wrap_2pi(encoder_get_angle_rad());
    elec_theta_m = wrap_2pi(mech_theta_m*mot_n_poles-elec_theta_bias);
}

static void update_estimates(float dt)
{
    const float tc = 0.0f;
    const float alpha = dt/(dt+tc);
    mech_omega_est += (wrap_pi(mech_theta_m-prev_mech_theta_m)/dt - mech_omega_est) * alpha;
    prev_mech_theta_m = mech_theta_m;

    // update the transformed current measurements
    transform_a_b_c_to_alpha_beta_gamma(ia_m, ib_m, ic_m, &ialpha_m, &ibeta_m, &igamma_m);
    transform_alpha_beta_to_d_q(ialpha_m, ibeta_m, &id_meas, &iq_meas);

    ekf_update(dt);
}

static void load_pid_configs(void)
{
    id_pid_param.K_R = iq_pid_param.K_R = curr_KR;
    id_pid_param.K_P = iq_pid_param.K_P = curr_KP;
    id_pid_param.K_I = iq_pid_param.K_I = curr_KI;
}

static void transform_a_b_c_to_alpha_beta_gamma(float a, float b, float c, float* alpha, float* beta, float* gamma)
{
    *alpha = 0.816496580927726f*a - 0.408248290463863f*b - 0.408248290463863f*c;
    *beta = 0.707106781186547f*b - 0.707106781186547f*c;
    *gamma = 0.577350269189626f*a + 0.577350269189626f*b + 0.577350269189626f*c;
}

static void transform_d_q_to_alpha_beta(float d, float q, float* alpha, float* beta)
{
    *alpha = d*cosf(elec_theta_m) - q*sinf(elec_theta_m);
    *beta = d*sinf(elec_theta_m) + q*cosf(elec_theta_m);
}

static void transform_alpha_beta_to_d_q(float alpha, float beta, float* d, float* q)
{
    *d = alpha*cosf(elec_theta_m) + beta*sinf(elec_theta_m);
    *q = -alpha*sinf(elec_theta_m) + beta*cosf(elec_theta_m);
}

static void svgen(float alpha, float beta, float* Va, float* Vb, float* Vc)
{
    // Per http://www.embedded.com/design/real-world-applications/4441150/2/Painless-MCU-implementation-of-space-vector-modulation-for-electric-motor-systems
    // Scaled such that overmodulation does not occur provided the magnitude of the input does not exceed max_duty.
    float Vneutral;

    (*Va) = alpha * 1.0f/sqrtf(3.0f);
    (*Vb) = (-(alpha/2.0f)+(beta*sqrtf(3.0f)/2.0f)) * 1.0f/sqrtf(3.0f);
    (*Vc) = (-(alpha/2.0f)-(beta*sqrtf(3.0f)/2.0f)) * 1.0f/sqrtf(3.0f);

    Vneutral = 0.5f * (MAX(MAX((*Va),(*Vb)),(*Vc)) + MIN(MIN((*Va),(*Vb)),(*Vc)));

    (*Va) += 0.5f*max_duty-Vneutral;
    (*Vb) += 0.5f*max_duty-Vneutral;
    (*Vc) += 0.5f*max_duty-Vneutral;

    (*Va) = constrain_float(*Va, 0.0f, max_duty);
    (*Vb) = constrain_float(*Vb, 0.0f, max_duty);
    (*Vc) = constrain_float(*Vc, 0.0f, max_duty);
}
