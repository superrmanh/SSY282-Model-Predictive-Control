import numpy as np 
import sys
# add path to the helper_funcs folder
sys.path.append("../helper_funcs")  # you might need to adjust the path

# Question 1: Norm problems as linear programs, and duality
#=============================================
print('\nQuestion 1: Norm problems as linear programs, and duality\n')

A = np.array([[0.4889, 0.2939],
     [1.0347, -0.7873],
     [0.7269, 0.8884],
     [-0.3034, -1.1471]])
b = np.array([-1.0689,-0.8095,-2.9443,1.4384]).reshape(-1,1)

c_sol = 'Vector in the cost function'
F_sol = 'Matrix in the inequality constraints'
g_sol = 'Vector in the inequality constraints'

# Question 2: Quadratic programming and KKT conditions
#=============================================
print('\nQuestion 2: Quadratic programming and KKT conditions\n')

# a) condensed form
Qbar_a_sol = 'Matrix in the cost function'
p_a_sol = 'Vector in the cost function'
K_a_sol = 'Optimal control policy'

# b) lifted form, QP qith equality constraints
Qbar_b_sol = 'Matrix in the cost function'
p_b_sol = 'Vector in the cost function' 
A_eq_sol = 'Matrix in the equality constraints'
b_eq_sol = 'Vector in the equality constraints'

K_b_sol = 'Optimal control policy'
L_b_sol = 'Optimal policy for the Lagrange multipliers'

# c) condensed form, QP qith inequality constraints
Qbar_c_sol = 'Matrix in the cost function'
p_c_sol = 'Vector in the cost function'
G_sol = 'Matrix in the inequality constraints'
h_sol = 'Vector in the inequality constraints'
u_sol = 'Optimal control inputs'
mu_sol = 'Optimal Lagrange multipliers'

# Question 3: Receding horizon control and Lagrange multipliers
#=============================================
print('\nQuestion 3: Receding horizon control and Lagrange multipliers\n')
h = 0.025 # sampling interval 
A = np.array([[1.0000,    0.0250,    0.0002,    0.0000],
         [0,     1.0000,    0.0180,    0.0002],
         [0,         0,     1.0049,    0.0250],
         [0,         0,     0.3954,    1.0049]])
B = np.array([0.0003,    0.0244,    0.0005,    0.0366]).reshape(-1,1)
Q = np.eye(4)
Pf = 10*np.eye(4)
Nsim = 400 # number of simulation instances
x0 = np.array([-0.5, 0, np.pi/12, 0]) # initial state
R = 1
N = 80

K_sol = 'Control feedback gain'
L_sol = 'The multipliers'' gain'
