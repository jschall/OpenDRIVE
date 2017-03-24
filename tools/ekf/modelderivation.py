from sympy import *
from sympy.physics.vector import dynamicsymbols

def copy_upper_to_lower_offdiagonals(M):
    assert isinstance(M,MatrixBase) and M.rows == M.cols

    ret = M[:,:]

    for r in range(ret.rows):
        for c in range(ret.cols):
            if r > c:
                ret[r,c] = ret[c,r]
    return ret

theta_e = dynamicsymbols('theta_e')

T_abc_aby = sqrt(2)/sqrt(3)*Matrix([[ S.One     , -S.Half   , -S.Half    ],
                                    [ S.Zero    , sqrt(3)/2 , -sqrt(3)/2 ],
                                    [ 1/sqrt(2) , 1/sqrt(2) , 1/sqrt(2)  ]])

T_aby_dqo = Matrix([[ cos(theta_e), sin(theta_e), S.Zero],
                    [-sin(theta_e), cos(theta_e), S.Zero],
                    [     S.Zero,     S.Zero,  S.One]])

T_abc_dqo = T_aby_dqo*T_abc_aby

T_dqo_abc = T_abc_aby.inv()*T_aby_dqo.T

I_abc = Matrix(dynamicsymbols('I_(a:c)'))
lambda_m = Symbol('lambda_m')
omega_e = Symbol('omega_e')
L_so, L_sl, L_x = symbols('L_so L_sl L_x')
R_s = Symbol('R_s')
t = Symbol('t')
lambda_dqo_sym = Matrix(dynamicsymbols('lambda_d lambda_q lambda_o'))
I_dqo_sym = Matrix(dynamicsymbols('I_d I_q I_o'))

# 2.13 .. 2.15
lambda_m_abc = Matrix([
    lambda_m*cos(theta_e),
    lambda_m*cos(theta_e-2*pi/3),
    lambda_m*cos(theta_e+2*pi/3)
    ])

# 2.7 .. 2.12
L_matrix = zeros(3,3)

L_matrix[0,0] = L_so + L_sl + L_x * cos(2*theta_e)
L_matrix[1,1] = L_so + L_sl + L_x * cos(2*theta_e + 2*pi/3)
L_matrix[2,2] = L_so + L_sl + L_x * cos(2*theta_e - 2*pi/3)
L_matrix[0,1] = -L_so/2 + L_x * cos(2*theta_e - 2*pi/3)
L_matrix[1,2] = -L_so/2 + L_x * cos(2*theta_e)
L_matrix[0,2] = -L_so/2 + L_x * cos(2*theta_e + 2*pi/3)

L_matrix = copy_upper_to_lower_offdiagonals(L_matrix)

# 2.4 .. 2.6
lambda_abc = L_matrix * I_abc + lambda_m_abc
lambda_abc = lambda_abc.subs(dict(zip(I_abc, T_dqo_abc*I_dqo_sym)))

V_abc = R_s * I_abc + diff(lambda_abc,t)
V_abc = V_abc.subs(dict(zip(I_abc, T_dqo_abc*I_dqo_sym)))

V_dqo = simplify(T_abc_dqo * V_abc)

pprint(collect(collect(V_dqo[0], diff(I_dqo_sym[0], t)), I_dqo_sym[1]*diff(theta_e,t)))
pprint(collect(collect(V_dqo[1], diff(I_dqo_sym[1], t)), I_dqo_sym[0]*diff(theta_e,t)))
#I_dqo = T_abc_dqo * I_abc
#lambda_dqo = simplify(T_abc_dqo * lambda_abc)

##L_dq_sym = Matrix(symbols('L_d L_q'))
##pprint(simplify(solve([L_dq_sym[0]*I_dqo[0]+lambda_m-lambda_dqo[0], L_dq_sym[1]*I_dqo[1]-lambda_dqo[1]], L_dq_sym)))

##V_dqo = V_dqo.subs(dict(zip([3*(L_so+L_x)/2+L_sl, 3*(L_so-L_x)/2+L_sl],L_dq_sym)))
##V_dqo = V_dqo.subs(dict(zip([3*(L_so+L_x)/2+L_sl, 3*(L_so-L_x)/2+L_sl],L_dq_sym)))
#pprint(V_dqo[0].subs(3*L_so/2 + 3*L_x/2 + L_sl, 0))
#pprint(simplify(V_dqo))

#L_dq_sym = Matrix(symbols('L_d L_q'))
#L_dq = Matrix([L_so-L_x, L_so+L_x])

#pprint(solve(L_dq-L_dq_sym, [L_so, L_sl, L_x]))
