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
static float elec_theta_bias = 0.0f;
// TODO support reverse
static bool reverse = false;
static const float curr_KR = 0.0f;
static const float curr_KP = .1f;
static const float curr_KI = 350.0f;
static const float vsense_div = 20.0f;
static const float csa_G = 10.0f;
static const float csa_R = 0.001f;
// it takes approximately 50 timer clock cycles to sample the current sensors. PWM period is 2000 timer clock cycles
// TODO reconstruct current measurement to allow max_duty=1.0
static const float max_duty = 1.0f;
static const float calibration_voltage = 2.0f;

static uint32_t csa_meas_t_us = 0;
static float csa_cal[3] = {0.0f, 0.0f, 0.0f}; // current sense amplifier calibration
static float vbatt_m = 0.0f; // battery voltage
static float ia_m = 0.0f, ib_m = 0.0f, ic_m = 0.0f; // phase currents
static float i_alpha_m = 0.0f, i_beta_m = 0.0f; // alpha-beta-gamma (clarke) transform of phase currents
static float id_meas = 0.0f, iq_meas = 0.0f; // dqo transform of phase currents
static float mech_theta_m = 0.0f; // mechanical rotor angle
static float prev_mech_theta_m = 0.0f; // previous mechanical rotor angle for differentiation
static float elec_theta_m = 0.0f; // electrical rotor angle
static float elec_theta_est = 0.0f;
static float elec_omega_est = 0.0f;
static float mech_omega_est = 0.0f; // mechanical rotor angular velocity
static float u_alpha = 0.0f, u_beta = 0.0f;
static enum motor_mode_t motor_mode = MOTOR_MODE_DISABLED;

// double-buffered phase output
static uint8_t phase_output_idx = 0;
static struct {
    uint32_t t_us;
    float duty_alpha;
    float duty_beta;
    float omega;
} phase_output[2];

static struct {
    uint32_t start_time_us;
    uint8_t step;
    float mech_theta_0;
} encoder_calibration_state;

static struct curr_pid_param_s iq_pid_param;
static struct curr_pid_state_s iq_pid_state;

static struct curr_pid_param_s id_pid_param;
static struct curr_pid_state_s id_pid_state;

static void process_adc_measurements(struct adc_sample_s* adc_sample);
static void retrieve_encoder_measurement(void);
static void update_estimates(float dt);
static void load_pid_configs(void);
static void transform_a_b_c_to_alpha_beta(float a, float b, float c, float* alpha, float* beta);
static void transform_d_q_to_alpha_beta(float theta, float d, float q, float* alpha, float* beta);
static void transform_alpha_beta_to_d_q(float theta, float alpha, float beta, float* d, float* q);
static void set_alpha_beta_output_duty(float duty_alpha, float duty_beta, float omega);
static void calc_phase_duties(float* phaseA, float* phaseB, float* phaseC);

static bool ekf_running = false;
static float state[5];
static float cov[15];

// EKF parameters
static const float R_s = 0.102f;
static const float L = 100*1e-6f;
static const float K_v = 360.0f;
static const float J = 0.00003f;
static const float N_P = 7.0f;
static const float i_noise = 0.01f;
static const float u_noise = 0.7f;
static const float T_l_pnoise = 0.1f;

static void ekf_init(float theta)
{
    ekf_running = true;
    state[0] = 0.0f; // omega
    state[1] = theta;
    state[2] = id_meas;
    state[3] = iq_meas;
    state[4] = 0;
    // 0 1 2 3
    //   4 5 6
    //     7 8
    //       9
    cov[0] = 100.000000000000;
    cov[1] = 0;
    cov[2] = 0;
    cov[3] = 0;
    cov[4] = 0;
    cov[5] = 9.86960440109;
    cov[6] = 0;
    cov[7] = 0;
    cov[8] = 0;
    cov[9] = ((i_noise)*(i_noise));
    cov[10] = 0;
    cov[11] = 0;
    cov[12] = ((i_noise)*(i_noise));
    cov[13] = 0;
    cov[14] = 0.0100000000000000;
}

static void ekf_update(float dt)
{
    if (!ekf_running) {
        return;
    }
    static float subx[102];
    static float state_n[5];
    static float cov_n[15];

    subx[0] = 1.0/J;
    subx[1] = 1.0/M_PI;
    subx[2] = 1.0/K_v;
    subx[3] = 30.0*subx[1]*subx[2];
    subx[4] = 1.0/L;
    subx[5] = N_P*dt;
    subx[6] = state[0]*subx[5];
    subx[7] = state[1] + subx[6];
    subx[8] = cosf(subx[7]);
    subx[9] = subx[8]*u_alpha;
    subx[10] = sinf(subx[7]);
    subx[11] = subx[10]*u_beta;
    subx[12] = N_P*state[3];
    subx[13] = R_s*subx[4];
    subx[14] = dt*(state[0]*subx[12] - state[2]*subx[13] + subx[4]*(subx[11] + subx[9])) + state[2];
    subx[15] = subx[14]*subx[8];
    subx[16] = subx[8]*u_beta;
    subx[17] = subx[10]*u_alpha;
    subx[18] = subx[16] - subx[17];
    subx[19] = N_P*state[2];
    subx[20] = 20.0*subx[1]*subx[2]*subx[4];
    subx[21] = dt*(-state[0]*subx[19] - state[0]*subx[20] - state[3]*subx[13] + subx[18]*subx[4]) + state[3];
    subx[22] = subx[10]*subx[21];
    subx[23] = subx[15] - subx[22];
    subx[24] = cov[4]*subx[5] + cov[8];
    subx[25] = dt*subx[0];
    subx[26] = 30.0*dt*subx[0]*subx[1]*subx[2];
    subx[27] = cov[0]*subx[5] + cov[1];
    subx[28] = -subx[24]*subx[25] + subx[26]*(cov[3]*subx[5] + cov[7]) + subx[27];
    subx[29] = -dt*subx[13] + 1;
    subx[30] = dt*(subx[12] + subx[4]*(subx[16]*subx[5] - subx[17]*subx[5]));
    subx[31] = dt*subx[18]*subx[4];
    subx[32] = cov[11]*subx[29] + cov[13]*subx[6] + cov[4]*subx[30] + cov[8]*subx[31];
    subx[33] = cov[10]*subx[29];
    subx[34] = cov[12]*subx[6] + cov[3]*subx[30] + cov[7]*subx[31] + subx[33];
    subx[35] = cov[0]*subx[30] + cov[1]*subx[31] + cov[2]*subx[29] + cov[3]*subx[6];
    subx[36] = -subx[25]*subx[32] + subx[26]*subx[34] + subx[35];
    subx[37] = dt*(-subx[19] - subx[20] + subx[4]*(-subx[11]*subx[5] - subx[5]*subx[9]));
    subx[38] = dt*subx[4]*(-subx[11] - subx[9]);
    subx[39] = -cov[11]*subx[6] + cov[13]*subx[29] + cov[4]*subx[37] + cov[8]*subx[38];
    subx[40] = cov[10]*subx[6];
    subx[41] = cov[12]*subx[29] + cov[3]*subx[37] + cov[7]*subx[38] - subx[40];
    subx[42] = cov[0]*subx[37] + cov[1]*subx[38] - cov[2]*subx[6] + cov[3]*subx[29];
    subx[43] = -subx[25]*subx[39] + subx[26]*subx[41] + subx[42];
    subx[44] = subx[10]*subx[36] + subx[23]*subx[28] + subx[43]*subx[8];
    subx[45] = -subx[10]*subx[14] - subx[21]*subx[8];
    subx[46] = cov[1]*subx[5] + cov[5] + subx[27]*subx[5];
    subx[47] = cov[1]*subx[30] + cov[5]*subx[31] + cov[6]*subx[29] + cov[7]*subx[6];
    subx[48] = subx[35]*subx[5] + subx[47];
    subx[49] = cov[1]*subx[37] + cov[5]*subx[38] - cov[6]*subx[6] + cov[7]*subx[29];
    subx[50] = subx[42]*subx[5] + subx[49];
    subx[51] = subx[10]*subx[48] + subx[23]*subx[46] + subx[50]*subx[8];
    subx[52] = cov[2]*subx[37] + cov[6]*subx[38] - cov[9]*subx[6] + subx[33];
    subx[53] = subx[29]*subx[52] + subx[30]*subx[42] + subx[31]*subx[49] + subx[41]*subx[6];
    subx[54] = subx[53]*subx[8];
    subx[55] = ((dt)*(dt))*((u_noise)*(u_noise))/((L)*(L));
    subx[56] = ((subx[10])*(subx[10]))*subx[55] + subx[55]*((subx[8])*(subx[8]));
    subx[57] = subx[29]*(cov[2]*subx[30] + cov[6]*subx[31] + cov[9]*subx[29] + subx[40]) + subx[30]*subx[35] + subx[31]*subx[47] + subx[34]*subx[6] + subx[56];
    subx[58] = subx[10]*subx[57] + subx[23]*subx[48] + subx[54];
    subx[59] = subx[10]*subx[53];
    subx[60] = subx[29]*subx[41] + subx[37]*subx[42] + subx[38]*subx[49] - subx[52]*subx[6] + subx[56];
    subx[61] = subx[23]*subx[50] + subx[59] + subx[60]*subx[8];
    subx[62] = -subx[10]*subx[61] + subx[45]*subx[51] + subx[58]*subx[8];
    subx[63] = ((i_noise)*(i_noise));
    subx[64] = subx[10]*subx[58] + subx[23]*subx[51] + subx[61]*subx[8] + subx[63];
    subx[65] = -subx[10]*subx[50] + subx[45]*subx[46] + subx[48]*subx[8];
    subx[66] = subx[45]*subx[48] + subx[57]*subx[8] - subx[59];
    subx[67] = -subx[10]*subx[60] + subx[45]*subx[50] + subx[54];
    subx[68] = -subx[10]*subx[67] + subx[45]*subx[65] + subx[63] + subx[66]*subx[8];
    subx[69] = 1.0/(-((subx[62])*(subx[62])) + subx[64]*subx[68]);
    subx[70] = subx[68]*subx[69];
    subx[71] = -subx[10]*subx[43] + subx[28]*subx[45] + subx[36]*subx[8];
    subx[72] = subx[62]*subx[69];
    subx[73] = subx[44]*subx[70] - subx[71]*subx[72];
    subx[74] = i_beta_m + subx[45];
    subx[75] = subx[64]*subx[69];
    subx[76] = -subx[44]*subx[72] + subx[71]*subx[75];
    subx[77] = i_alpha_m - subx[15] + subx[22];
    subx[78] = subx[51]*subx[70] - subx[65]*subx[72];
    subx[79] = -subx[51]*subx[72] + subx[65]*subx[75];
    subx[80] = subx[58]*subx[70] - subx[66]*subx[72];
    subx[81] = -subx[58]*subx[72] + subx[66]*subx[75];
    subx[82] = subx[61]*subx[70] - subx[67]*subx[72];
    subx[83] = -subx[61]*subx[72] + subx[67]*subx[75];
    subx[84] = subx[10]*subx[32] + subx[23]*subx[24] + subx[39]*subx[8];
    subx[85] = -subx[10]*subx[39] + subx[24]*subx[45] + subx[32]*subx[8];
    subx[86] = subx[70]*subx[84] - subx[72]*subx[85];
    subx[87] = -subx[72]*subx[84] + subx[75]*subx[85];
    subx[88] = subx[10]*subx[76] - subx[73]*subx[8];
    subx[89] = -subx[23]*subx[73] - subx[45]*subx[76];
    subx[90] = -subx[10]*subx[73] - subx[76]*subx[8];
    subx[91] = cov[13]*subx[25];
    subx[92] = -cov[14]*subx[25] + cov[4] + subx[3]*subx[91];
    subx[93] = subx[10]*subx[79] - subx[78]*subx[8];
    subx[94] = -subx[10]*subx[78] - subx[79]*subx[8];
    subx[95] = -subx[23]*subx[78] - subx[45]*subx[79] + 1;
    subx[96] = subx[10]*subx[81] - subx[80]*subx[8];
    subx[97] = -subx[23]*subx[80] - subx[45]*subx[81];
    subx[98] = -subx[10]*subx[80] - subx[81]*subx[8] + 1;
    subx[99] = -subx[23]*subx[82] - subx[45]*subx[83];
    subx[100] = -subx[10]*subx[82] - subx[83]*subx[8];
    subx[101] = subx[10]*subx[83] - subx[82]*subx[8] + 1;
    state_n[0] = dt*(state[3]*subx[0]*subx[3] - state[4]*subx[0]) + state[0] + subx[73]*subx[74] + subx[76]*subx[77];
    state_n[1] = subx[74]*subx[78] + subx[77]*subx[79] + subx[7];
    state_n[2] = subx[14] + subx[74]*subx[80] + subx[77]*subx[81];
    state_n[3] = subx[21] + subx[74]*subx[82] + subx[77]*subx[83];
    state_n[4] = state[4] + subx[74]*subx[86] + subx[77]*subx[87];
    cov_n[0] = cov[0] + cov[3]*subx[26] - cov[4]*subx[25] - subx[25]*subx[92] + subx[26]*(cov[12]*subx[26] + cov[3] - subx[91]) + subx[28]*subx[89] + subx[36]*subx[90] + subx[43]*subx[88];
    cov_n[1] = subx[28] + subx[46]*subx[89] + subx[48]*subx[90] + subx[50]*subx[88];
    cov_n[2] = subx[36] + subx[48]*subx[89] + subx[53]*subx[88] + subx[57]*subx[90];
    cov_n[3] = subx[43] + subx[50]*subx[89] + subx[53]*subx[90] + subx[60]*subx[88];
    cov_n[4] = subx[24]*subx[89] + subx[32]*subx[90] + subx[39]*subx[88] + subx[92];
    cov_n[5] = subx[46]*subx[95] + subx[48]*subx[94] + subx[50]*subx[93];
    cov_n[6] = subx[48]*subx[95] + subx[53]*subx[93] + subx[57]*subx[94];
    cov_n[7] = subx[50]*subx[95] + subx[53]*subx[94] + subx[60]*subx[93];
    cov_n[8] = subx[24]*subx[95] + subx[32]*subx[94] + subx[39]*subx[93];
    cov_n[9] = subx[48]*subx[97] + subx[53]*subx[96] + subx[57]*subx[98];
    cov_n[10] = subx[50]*subx[97] + subx[53]*subx[98] + subx[60]*subx[96];
    cov_n[11] = subx[24]*subx[97] + subx[32]*subx[98] + subx[39]*subx[96];
    cov_n[12] = subx[100]*subx[53] + subx[101]*subx[60] + subx[50]*subx[99];
    cov_n[13] = subx[100]*subx[32] + subx[101]*subx[39] + subx[24]*subx[99];
    cov_n[14] = ((T_l_pnoise)*(T_l_pnoise)) + cov[14] + subx[24]*(-subx[23]*subx[86] - subx[45]*subx[87]) + subx[32]*(-subx[10]*subx[86] - subx[87]*subx[8]) + subx[39]*(subx[10]*subx[87] - subx[86]*subx[8]);

    state_n[1] = wrap_2pi(state_n[1]);

    memcpy(state, state_n, sizeof(state));
    memcpy(cov, cov_n, sizeof(cov));
}

void motor_print_data(float dt) {
    uint8_t slip_msg[64];
    uint8_t slip_msg_len = 0;
    uint8_t i;
    uint32_t tnow_us = micros();
    float omega_e = mech_omega_est*mot_n_poles;

//     for (i=0; i<sizeof(float); i++) {
//         slip_encode_and_append(((uint8_t*)&(state[1]))[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
//     }

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
        slip_encode_and_append(((uint8_t*)&i_alpha_m)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&i_beta_m)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&u_alpha)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    for (i=0; i<sizeof(float); i++) {
        slip_encode_and_append(((uint8_t*)&u_beta)[i], &slip_msg_len, slip_msg, sizeof(slip_msg));
    }

    slip_msg[slip_msg_len++] = SLIP_END;

//     serial_send_dma(slip_msg_len, (char*)slip_msg);
}

void motor_init(void)
{
    // calibrate phase currents
    uint16_t i;
    struct adc_sample_s adc_sample;
    drv_csa_cal_mode_on();
    usleep(50);
    memset(csa_cal, 0, sizeof(csa_cal));
    for(i=0; i<1000; i++) {
        adc_wait_for_sample();
        adc_get_sample(&adc_sample);
        csa_cal[0] += adc_sample.csa_v[0];
        csa_cal[1] += adc_sample.csa_v[1];
        csa_cal[2] += adc_sample.csa_v[2];
    }
    drv_csa_cal_mode_off();
    csa_cal[0] /= 1000;
    csa_cal[1] /= 1000;
    csa_cal[2] /= 1000;

    // initialize encoder filter states
    retrieve_encoder_measurement();
    prev_mech_theta_m = mech_theta_m;
    mech_omega_est = 0.0f;

    pwm_set_phase_duty_callback(calc_phase_duties);
}

void motor_update_state(float dt, struct adc_sample_s* adc_sample)
{
    retrieve_encoder_measurement();
    process_adc_measurements(adc_sample);
    update_estimates(dt);
}

void motor_run_commutation(float dt)
{
    load_pid_configs();

    id_pid_param.dt = iq_pid_param.dt  = dt;

    id_pid_param.i_meas = id_meas;
    iq_pid_param.i_meas = iq_meas;

    switch (motor_mode) {
        case MOTOR_MODE_DISABLED:
            break;

        case MOTOR_MODE_FOC_CURRENT:
            id_pid_param.i_ref = 0.0f;
            id_pid_param.output_limit = max_duty*vbatt_m / sqrtf(2);
            curr_pid_run(&id_pid_param, &id_pid_state);

            // limit iq such that
            iq_pid_param.output_limit = sqrtf(MAX(SQ(id_pid_param.output_limit)-SQ(id_pid_state.output),0));
            curr_pid_run(&iq_pid_param, &iq_pid_state);

            transform_d_q_to_alpha_beta(elec_theta_est, id_pid_state.output, iq_pid_state.output, &u_alpha, &u_beta);

            set_alpha_beta_output_duty(u_alpha/vbatt_m, u_beta/vbatt_m, elec_omega_est);
            break;

        case MOTOR_MODE_ENCODER_CALIBRATION: {
            float t = (micros() - encoder_calibration_state.start_time_us)*1.0e-6f;
            float theta = 0.0f;

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
                    semihost_debug_printf("mot_n_poles %u rev %u\n", mot_n_poles, reverse);
                    break;
            }

            float v = constrain_float(calibration_voltage, 0.0f, max_duty*vbatt_m);
            u_alpha = v * cosf(theta);
            u_beta = v * sinf(theta);

            set_alpha_beta_output_duty(u_alpha/vbatt_m, u_beta/vbatt_m, 0);

            break;
        }

        case MOTOR_MODE_PHASE_VOLTAGE_TEST: {
            float theta = wrap_2pi(millis()*1e-3f);

            float v = constrain_float(calibration_voltage, 0.0f, max_duty*vbatt_m);
            u_alpha = v * cosf(theta);
            u_beta = v * sinf(theta);

            set_alpha_beta_output_duty(u_alpha/vbatt_m, u_beta/vbatt_m, 0);
            break;
        }
    }
}

void motor_set_mode(enum motor_mode_t new_mode)
{
    if (new_mode == motor_mode) {
        return;
    }

    if (new_mode == MOTOR_MODE_DISABLED) {
        drv_6_pwm_mode();
    } else {
        drv_3_pwm_mode();
    }

    if (new_mode == MOTOR_MODE_ENCODER_CALIBRATION) {
        encoder_calibration_state.start_time_us = micros();
        encoder_calibration_state.step = 0;
    }

    // reset PID states and inputs
    memset(&id_pid_state,0,sizeof(id_pid_state));
    memset(&iq_pid_state,0,sizeof(iq_pid_state));
    id_pid_param.i_ref = 0.0f;
    iq_pid_param.i_ref = 0.0f;

    if (new_mode == MOTOR_MODE_FOC_CURRENT) {
        ekf_init(0);
//         semihost_debug_printf("%d %d\n", (int32_t)(100.0*180.0/M_PI_F * elec_theta_m), (int32_t)(100.0*180.0/M_PI_F * state[1]));
    }

    motor_mode = new_mode;
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

static void process_adc_measurements(struct adc_sample_s* adc_sample)
{
    // Retrieve battery measurement
    vbatt_m = adc_sample->vsense_v * vsense_div;

    // Retrieve current sense amplifier measurement
    csa_meas_t_us = adc_sample->t_us;
    ia_m = (adc_sample->csa_v[0]-csa_cal[0])/(csa_G*csa_R);
    ib_m = (adc_sample->csa_v[1]-csa_cal[1])/(csa_G*csa_R);
    ic_m = (adc_sample->csa_v[2]-csa_cal[2])/(csa_G*csa_R);

    // Reconstruct current measurement
    float duty_a, duty_b, duty_c;
    pwm_get_phase_duty(&duty_a, &duty_b, &duty_c);

    if (duty_a > duty_b && duty_a > duty_c) {
        ia_m = -ib_m-ic_m;
    } else if (duty_b > duty_a && duty_b > duty_c) {
        ib_m = -ia_m-ic_m;
    } else {
        ic_m = -ia_m-ib_m;
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

    transform_a_b_c_to_alpha_beta(ia_m, ib_m, ic_m, &i_alpha_m, &i_beta_m);

    ekf_update(dt);

    if (ekf_running) {
        elec_theta_est = state[1];
        elec_omega_est = state[0] * mot_n_poles;
    } else {
        elec_theta_est = elec_theta_m;
        elec_omega_est = mech_omega_est * mot_n_poles;
    }

    transform_alpha_beta_to_d_q(elec_theta_est, i_alpha_m, i_beta_m, &id_meas, &iq_meas);
}

static void load_pid_configs(void)
{
    id_pid_param.K_R = iq_pid_param.K_R = curr_KR;
    id_pid_param.K_P = iq_pid_param.K_P = curr_KP;
    id_pid_param.K_I = iq_pid_param.K_I = curr_KI;
}

static void transform_a_b_c_to_alpha_beta(float a, float b, float c, float* alpha, float* beta)
{
    *alpha = 0.816496580927726f*a - 0.408248290463863f*b - 0.408248290463863f*c;
    *beta = 0.707106781186547f*b - 0.707106781186547f*c;
}

static void transform_d_q_to_alpha_beta_fast(float theta, float d, float q, float* alpha, float* beta)
{
    float sin_theta = theta;
    float cos_theta = 1.0f-0.5f*SQ(theta);
    *alpha = d*cos_theta - q*sin_theta;
    *beta = d*sin_theta + q*cos_theta;
}

static void transform_d_q_to_alpha_beta(float theta, float d, float q, float* alpha, float* beta)
{
    *alpha = d*cosf(theta) - q*sinf(theta);
    *beta = d*sinf(theta) + q*cosf(theta);
}

static void transform_alpha_beta_to_d_q(float theta, float alpha, float beta, float* d, float* q)
{
    *d = alpha*cosf(theta) + beta*sinf(theta);
    *q = -alpha*sinf(theta) + beta*cosf(theta);
}

static void set_alpha_beta_output_duty(float duty_alpha, float duty_beta, float omega)
{
    uint8_t next_phase_output_idx = (phase_output_idx+1)%2;
    phase_output[next_phase_output_idx].t_us = csa_meas_t_us;
    phase_output[next_phase_output_idx].omega = omega;
    phase_output[next_phase_output_idx].duty_alpha = duty_alpha;
    phase_output[next_phase_output_idx].duty_beta = duty_beta;
    phase_output_idx = next_phase_output_idx;
}

static volatile float delta_theta;
static void calc_phase_duties(float* phaseA, float* phaseB, float* phaseC)
{
    if (motor_mode == MOTOR_MODE_DISABLED) {
        (*phaseA) = 0;
        (*phaseB) = 0;
        (*phaseC) = 0;
        return;
    }

    uint32_t tnow_us = micros();

    float alpha, beta;
    alpha = phase_output[phase_output_idx].duty_alpha;
    beta = phase_output[phase_output_idx].duty_beta;
//     delta_theta = (tnow_us-phase_output[phase_output_idx].t_us)*1e-6f * phase_output[phase_output_idx].omega;
//     transform_d_q_to_alpha_beta_fast(delta_theta, phase_output[phase_output_idx].duty_alpha, phase_output[phase_output_idx].duty_beta, &alpha, &beta);

    // Space-vector generator
    // Per http://www.embedded.com/design/real-world-applications/4441150/2/Painless-MCU-implementation-of-space-vector-modulation-for-electric-motor-systems
    float Vneutral;
    // Does not overmodulate, provided the input magnitude is <= max_duty/sqrt(2)

    (*phaseA) = alpha * sqrtf(2.0f/3.0f);
    (*phaseB) = (-(alpha/2.0f)+(beta*sqrtf(3.0f)/2.0f)) * sqrtf(2.0f/3.0f);
    (*phaseC) = (-(alpha/2.0f)-(beta*sqrtf(3.0f)/2.0f)) * sqrtf(2.0f/3.0f);

    Vneutral = 0.5f * (MAX(MAX((*phaseA),(*phaseB)),(*phaseC)) + MIN(MIN((*phaseA),(*phaseB)),(*phaseC)));

    (*phaseA) += 0.5f*max_duty-Vneutral;
    (*phaseB) += 0.5f*max_duty-Vneutral;
    (*phaseC) += 0.5f*max_duty-Vneutral;

    (*phaseA) = constrain_float(*phaseA, 0.0f, max_duty);
    (*phaseB) = constrain_float(*phaseB, 0.0f, max_duty);
    (*phaseC) = constrain_float(*phaseC, 0.0f, max_duty);
}
