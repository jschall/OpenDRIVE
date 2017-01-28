from sympy import *
from sympy.printing.ccode import *
from math import sqrt, floor, fmod
import math
from helpers import *
import sys

# Refs:
# "Estimation of Rotor Position and Speed for Sensorless DSP-based PMSM Drives," Ciabattoni, et al., 2011
# "Adaptive Extended Kalman Filter for Robust Sensorless Control of PMSM Drives," Ciabattoni, et al., 2011
#     http://www.nt.ntnu.no/users/skoge/prost/proceedings/cdc-ecc-2011/data/papers/1796.pdf
# "Extended Kalman Filter Application in Permanent Magnet Synchronous Motor Sensorless Control," Qiu, 2003
# "Extended Kalman Filter Based Speed Sensorless PMSM Control with Load Reconstruction," Janiszewski, 2010
# "Dynamic Model of PM Synchronous Motors," Ohm, 2000
#     http://www.drivetechinc.com/articles/IM97PM_Rev1forPDF.pdf
# "AC Drive Observability Analysis"
#     https://www.ceitec.cz/rp2-ac-drive-observability-analysis/f1220

# Parameters
dt  = Symbol('dt')  # Time step
R_s = Symbol('R_s') # Stator resistance
L_d = Symbol('L_d')   # L_d
L_q = Symbol('L_q')   # L_q
K_v = Symbol('K_v') # Motor back-emf constant, RPM/V
N_P = Symbol('N_P') # Number of magnetic pole pairs
J   = Symbol('J')   # Rotor inertia
T_l_pnoise = Symbol('T_l_pnoise') # Load torque process noise
i_pnoise = Symbol('i_pnoise') # Current process noise
omega_pnoise = Symbol('omega_pnoise')
theta_pnoise = Symbol('theta_pnoise')
K_t = 30./(K_v*pi) # Motor torque constant.
K_m = K_t/N_P
lambda_r = 2./3. * K_t/N_P # Rotor flux linkage - Ohm, section III, eqn 3.7

# Inputs
u_ab = Matrix(symbols('u_alpha u_beta')) # Stator voltages
u_noise = Symbol('u_noise') # Additive noise on stator voltage
w_u_sigma = Matrix([u_noise, u_noise]) # Additive noise on u
Q_u = diag(*w_u_sigma.multiply_elementwise(w_u_sigma)) # Covariance of additive noise on u

# Measurements
i_ab_m = Matrix(symbols('i_alpha_m i_beta_m')) # Stator currents observed
i_noise = Symbol('i_noise') # Additive noise on stator currents
z = toVec(i_ab_m) # Observation vector
R = diag(i_noise**2,i_noise**2) # Covariance of observation vector

# States
omega_r, theta_e, i_alpha, i_beta, T_l_est = symbols('state[0:5]')
x = toVec(omega_r, theta_e, i_alpha, i_beta, T_l_est)
nStates = len(x)

# Covariance matrix
P = compressedSymmetricMatrix('cov', nStates)

# Derived variables
omega_e = omega_r*N_P
theta_r = theta_e/N_P

R_ab_dq = lambda theta: Matrix([[ cos(theta), sin(theta)],
                                [-sin(theta), cos(theta)]])
R_dq_ab = lambda theta: R_ab_dq(theta).T

i_ab = Matrix([i_alpha, i_beta])
i_dq = R_ab_dq(theta_e)*i_ab
i_d, i_q = i_dq[0], i_dq[1]

u_alpha, u_beta = u_ab[0], u_ab[1]

i_alpha_dot = (-L_d*L_q*i_beta*omega_e + L_d*(lambda_r*omega_e + L_d*i_alpha*omega_e*cos(theta_e) + L_d*i_beta*omega_e*sin(theta_e) - R_s*i_alpha*sin(theta_e) + R_s*i_beta*cos(theta_e) + u_alpha*sin(theta_e) - u_beta*cos(theta_e))*sin(theta_e) + L_q*(-L_q*i_alpha*omega_e*sin(2*theta_e) + L_q*i_beta*omega_e*cos(2*theta_e) + L_q*i_beta*omega_e - R_s*i_alpha*cos(2*theta_e) - R_s*i_alpha - R_s*i_beta*sin(2*theta_e) + u_alpha*cos(2*theta_e) + u_alpha + u_beta*sin(2*theta_e))/2)/(L_d*L_q)

i_beta_dot = (2*L_d*L_q*i_alpha*omega_e + L_d*(-2*lambda_r*omega_e*cos(theta_e) - L_d*i_alpha*omega_e*cos(2*theta_e) - L_d*i_alpha*omega_e - L_d*i_beta*omega_e*sin(2*theta_e) + R_s*i_alpha*sin(2*theta_e) - R_s*i_beta*cos(2*theta_e) - R_s*i_beta - u_alpha*sin(2*theta_e) + u_beta*cos(2*theta_e) + u_beta) + L_q*(L_q*i_alpha*omega_e*cos(2*theta_e) - L_q*i_alpha*omega_e + L_q*i_beta*omega_e*sin(2*theta_e) - R_s*i_alpha*sin(2*theta_e) + R_s*i_beta*cos(2*theta_e) - R_s*i_beta + u_alpha*sin(2*theta_e) - u_beta*cos(2*theta_e) + u_beta))/(2*L_d*L_q)

omega_r_dot = i_q*K_t/J + (L_d-L_q)*i_q*i_d - T_l_est/J

# f: state-transtition model
f = Matrix([
    [omega_r + dt*omega_r_dot],
    [theta_e + dt*omega_e],
    [i_alpha + dt*i_alpha_dot],
    [i_beta + dt*i_beta_dot],
    [T_l_est]
    ])
assert f.shape == x.shape

# F: linearized state-transition model, AKA "A" in literature
F = f.jacobian(x)

# u: control input vector
u = u_ab

# G: control-influence matrix, AKA "B" in literature
G = f.jacobian(u)

# Q: covariance of additive noise on x
Q = G*Q_u*G.T
Q += diag(0**2, 0**2, 0**2, 0**2, T_l_pnoise**2)

x_p = f

# P_p: covariance matrix at time k+1
P_p = F*P*F.T + Q
assert P_p.shape == P.shape

# h: predicted measurement
h = i_ab

# y: innovation vector
y = z-h

# H: measurement sensitivity matrix
H = h.jacobian(x)

# S: innovation covariance
S = H*P*H.T + R

S_I = quickinv_sym(S)

# K: Kalman gain
K = P*H.T*S_I

I = eye(nStates)

NIS = (y.T*S_I*y).xreplace(dict(zip(x,x_p)+zip(P,P_p))) # normalized innovation squared

x_n = (x + K*y).xreplace(dict(zip(x,x_p)+zip(P,P_p)))
P_n = ((I-K*H)*P).xreplace(dict(zip(x,x_p)+zip(P,P_p)))

P_p = upperTriangularToVec(P_p)
P_n = upperTriangularToVec(P_n)

def print_code():
    global x_n, P_n
    #subs = [
        #(sin(theta_e), Symbol('ekf_sin_theta')),
        #(cos(theta_e), Symbol('ekf_cos_theta')),
        #(sin(theta_e+dt*omega_e), Symbol('next_ekf_sin_theta')),
        #(cos(theta_e+dt*omega_e), Symbol('next_ekf_cos_theta'))
        #]
    #x_n = x_n.subs(subs)
    #P_n = P_n.subs(subs)


    x_n,P_n,subx = extractSubexpressions([x_n,P_n],'subx',threshold=3)
    print count_ops(x_n)+count_ops(P_n)+count_ops(subx)

    init_P = upperTriangularToVec(diag(100., math.pi**2, i_noise**2, i_noise**2, 0.1**2))

    for i in range(len(init_P)):
        print('cov[%u] = %s;' % (i, CCodePrinter_float().doprint(init_P[i])))

    for i in range(len(subx)):
        print('%s = %s;' % (subx[i][0], CCodePrinter_float().doprint(subx[i][1])))

    for i in range(len(x_n)):
        print('state_n[%u] = %s;' % (i, CCodePrinter_float().doprint(x_n[i])))

    for i in range(len(P_n)):
        print('cov_n[%u] = %s;' % (i, CCodePrinter_float().doprint(P_n[i])))

def test_ekf():
    global x_n, P_n, dt, NIS, S, y
    from scipy.io import loadmat
    import numpy as np
    data = loadmat('mot_data.mat')

    PREDICTION_ONLY = False
    TRUTH_ANGLE_OVERRIDE = False

    if PREDICTION_ONLY:
        print("PREDICTION_ONLY")
    if TRUTH_ANGLE_OVERRIDE:
        print("TRUTH_ANGLE_OVERRIDE")

    if PREDICTION_ONLY:
        x_n = x_p
        P_n = P_p

    subs = {
        R_s:0.102,
        L_d:45.0*1e-6,
        L_q:70.0*1e-6,
        K_v:360.,
        J:0.00003,
        N_P:7,
        i_noise: 0.06,
        u_noise: 0.5,
        T_l_pnoise: 25.*dt,
        }

    x_n = x_n.xreplace(subs).xreplace(subs)
    P_n = P_n.xreplace(subs).xreplace(subs)
    NIS = NIS.xreplace(subs).xreplace(subs)
    S = S.xreplace(subs).xreplace(subs)
    y = y.xreplace(subs).xreplace(subs)

    #pprint(Q.xreplace(subs).xreplace(subs))

    x_n,P_n,NIS,S,y,subx = extractSubexpressions([x_n,P_n,NIS,S,y],'subx',threshold=1)

    from sympy.utilities.autowrap import ufuncify
    lambda_args = (zip(*subx)[0], x, upperTriangularToVec(P), dt, i_ab_m[0], i_ab_m[1], u_ab[0], u_ab[1])

    subx_lambda = []

    for sym,expr in subx:
        subx_lambda.append(lambdify(lambda_args, expr))

    x_n_lambda = lambdify(lambda_args, x_n)
    P_n_lambda = lambdify(lambda_args, P_n)
    NIS_lambda = lambdify(lambda_args, NIS)
    S_lambda = lambdify(lambda_args, S)
    y_lambda = lambdify(lambda_args, y)

    init_P = upperTriangularToVec(diag(10.**2, math.pi**2, 0.01**2, 0.01**2, 0.1**2))

    curr_x = np.array([0.,data['theta_e'][0][0]+math.pi/4, 0., 0., 0.])
    curr_P = np.array(init_P.T)
    curr_subx = np.zeros(len(subx_lambda))


    plot_data = {}

    def add_plot_data(name, val):
        if name not in plot_data.keys():
            plot_data[name] = []
        plot_data[name].append(val)

    #data['u_alpha'][0] = np.roll(data['u_alpha'][0],1)
    #data['u_beta'][0] = np.roll(data['u_beta'][0],1)
    #data['u_alpha'][0][0] = data['u_beta'][0][0] = 0
    #data['u_alpha'][0][1] = data['u_beta'][0][1] = 0

    #print data['u_alpha'][0][0]
    #print data['u_beta'][0][0]

    n_samples = len(data['dt'][0])
    n_process = n_samples
    poison = False
    for i in range(n_process):
        try:
            t = (data['t_us'][0][i]-data['t_us'][0][0])*1e-6
            data['t_us'][0][i]
            sys.stdout.write('\r%fs %u%%' % (t,int(round(100*float(i)/n_process))))
            sys.stdout.flush()
            theta_e_truth = data['theta_e'][0][i]
            omega_e_truth = data['omega_e'][0][i]
            dt, i_alpha_m, i_beta_m, u_alpha, u_beta = (data['dt'][0][i], data['i_alpha'][0][i], data['i_beta'][0][i], data['u_alpha'][0][i], data['u_beta'][0][i])

            lambda_args = (curr_subx, curr_x, curr_P, dt, i_alpha_m, i_beta_m, u_alpha, u_beta)
            for j in range(len(subx)):
                curr_subx[j] = subx_lambda[j](*lambda_args)
            next_x = x_n_lambda(*lambda_args)
            next_P = P_n_lambda(*lambda_args)

            obs_S = S_lambda(*lambda_args)
            obs_y = y_lambda(*lambda_args)
            obs_NIS = NIS_lambda(*lambda_args)[0][0]

            if TRUTH_ANGLE_OVERRIDE:
                next_x[1][0] = theta_e_truth

            next_x[1][0] = fmod(next_x[1][0],2*math.pi)
            if next_x[1][0] < 0:
                next_x[1][0] += 2*math.pi

            next_P_uncompressed = uncompressSymMatrix(next_P)

            theta_e_mu = next_x[1][0]
            theta_e_sigma = float(next_P_uncompressed[1,1]**0.5)
            T_l_mu = next_x[4][0]
            T_l_sigma = float(next_P_uncompressed[4,4]**0.5)
            omega_e_mu = next_x[0][0]*7
            omega_e_sigma = float(next_P_uncompressed[0,0]**0.5 * 7)
            i_alpha_mu = next_x[2][0]
            i_alpha_sigma = float(next_P_uncompressed[2,2]**0.5)
            i_beta_mu = next_x[3][0]
            i_beta_sigma = float(next_P_uncompressed[3,3]**0.5)

            add_plot_data('t', t)

            add_plot_data('theta_e_est', theta_e_mu)
            add_plot_data('theta_e_est_min', theta_e_mu-theta_e_sigma)
            add_plot_data('theta_e_est_max', theta_e_mu+theta_e_sigma)
            add_plot_data('theta_e_truth',theta_e_truth)
            add_plot_data('theta_e_truth_min',theta_e_truth-0.1)
            add_plot_data('theta_e_truth_max',theta_e_truth+0.1)

            add_plot_data('omega_e_est',omega_e_mu)
            add_plot_data('omega_e_est_min',omega_e_mu-omega_e_sigma)
            add_plot_data('omega_e_est_max',omega_e_mu+omega_e_sigma)
            add_plot_data('omega_e_truth',omega_e_truth)
            add_plot_data('omega_e_truth_min',omega_e_truth-50.)
            add_plot_data('omega_e_truth_max',omega_e_truth+50.)

            add_plot_data('T_l_est',T_l_mu)
            add_plot_data('T_l_est_min',T_l_mu-T_l_sigma)
            add_plot_data('T_l_est_max',T_l_mu+T_l_sigma)

            add_plot_data('i_alpha_est', i_alpha_mu)
            add_plot_data('i_alpha_est_min', i_alpha_mu-i_alpha_sigma)
            add_plot_data('i_alpha_est_max', i_alpha_mu+i_alpha_sigma)
            add_plot_data('i_alpha_m', i_alpha_m)

            add_plot_data('i_beta_est', i_beta_mu)
            add_plot_data('i_beta_est_min', i_beta_mu-i_beta_sigma)
            add_plot_data('i_beta_est_max', i_beta_mu+i_beta_sigma)
            add_plot_data('i_beta_m', i_beta_m)

            add_plot_data('omega_e_err', omega_e_mu-omega_e_truth)

            add_plot_data('theta_e_err', wrap_pi(theta_e_mu-theta_e_truth))

            add_plot_data('NIS',obs_NIS)


            curr_x = next_x
            curr_P = next_P
            if poison:
                break
        except KeyboardInterrupt:
            poison=True

    sys.stdout.write('\n')
    import matplotlib.pyplot as plt

    plt.figure(1)
    plt.subplot(4,2,1)
    plt.title('electrical rotor angle')
    plt.fill_between(plot_data['t'], plot_data['theta_e_est_min'], plot_data['theta_e_est_max'], facecolor='b', alpha=0.25)
    plt.plot(plot_data['t'], plot_data['theta_e_est'], color='b')
    plt.plot(plot_data['t'], plot_data['theta_e_truth'], color='g')
    plt.subplot(4,2,2)
    plt.title('electrical rotor angular velocity')
    plt.fill_between(plot_data['t'], plot_data['omega_e_est_min'], plot_data['omega_e_est_max'], facecolor='b', alpha=0.25)
    plt.plot(plot_data['t'], plot_data['omega_e_est'], color='b')
    plt.plot(plot_data['t'], plot_data['omega_e_truth'], color='g')
    plt.subplot(4,2,3)
    plt.title('alpha-axis current')
    plt.fill_between(plot_data['t'], plot_data['i_alpha_est_min'], plot_data['i_alpha_est_max'], facecolor='b', alpha=0.25)
    plt.plot(plot_data['t'], plot_data['i_alpha_est'], color='b')
    plt.plot(plot_data['t'], plot_data['i_alpha_m'], color='g')
    plt.subplot(4,2,4)
    plt.title('beta-axis current')
    plt.fill_between(plot_data['t'], plot_data['i_beta_est_min'], plot_data['i_beta_est_max'], facecolor='b', alpha=0.25)
    plt.plot(plot_data['t'], plot_data['i_beta_est'], color='b')
    plt.plot(plot_data['t'], plot_data['i_beta_m'], color='g')
    plt.subplot(4,2,5)
    plt.title('load torque')
    plt.fill_between(plot_data['t'],plot_data['T_l_est_max'],plot_data['T_l_est_min'],facecolor='b',alpha=.25)
    plt.plot(plot_data['t'], plot_data['T_l_est'], color='b')
    plt.subplot(4,2,6)
    plt.title('current fusion normalized innovations squared')
    plt.plot(plot_data['t'], plot_data['NIS'])
    plt.subplot(4,2,7)
    plt.title('electrical rotor angle error vs sensor')
    plt.plot(plot_data['t'], plot_data['theta_e_err'])
    plt.subplot(4,2,8)
    plt.title('electrical rotor angular velocity error vs sensor')
    plt.plot(plot_data['t'], plot_data['omega_e_err'])

    plt.show()

print_code()
#test_ekf()
