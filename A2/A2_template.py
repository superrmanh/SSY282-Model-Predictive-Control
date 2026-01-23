import numpy as np 
from numpy.linalg import norm 
import matplotlib.pyplot as plt 
from scipy.linalg import expm, eig 
import sys
sys.path.append("../helper_funcs")  # you might need to adjust the path

# Questtion 1: Dynamic Programming solution of the LQ problem
#=============================================================
print('Question 1: Dynamic Programming solution of the LQ problem')

# a) Stabilizing controller through Dynamic Programming
# system matrices A, B,:
A = np.array([
    [1.0000,    0.0250,    0.0002,    0.0000],
    [ 0,     1.0000,    0.0180,    0.0002],
    [ 0,         0,     1.0049,    0.0250],
    [ 0,         0,     0.3954,    1.0049]])

B =np.array([0.0003,    0.0244,    0.0005,    0.0366]).reshape(-1,1)
Q = np.eye(4)
R = 1
Pf = 10*np.eye(4)

# Write your code


N_DP_sol ='The minimum horizon to give a stable closed-loop'
K_DP_sol ='The feedback gain'

print(f'The minimum horizon to give a stable closed-loop is N= {N_DP_sol}\n')
print('The feedback gain is K_DP=\n')
print(K_DP_sol)

# b) Stationary Riccati matrix
from scipy.linalg import solve_discrete_are

# Write your code

P_inf_dare_sol = 'The stationary Riccati matrix using dare'
P_inf_DP_sol = 'The stationary Riccati matrix from DP'
N_inf_DP_sol = 'The number of DP iterations until convergence'

print('\nThe stationary Riccati matrix using idare is \n')
print('Pf_idare_sol = \n',P_inf_dare_sol); 

print('The stationary Riccati matrix from DP is \n')
print('P_inf_DP= \n ',P_inf_DP_sol); 

print(f'\nThe number of DP iterations until convergence is \nN_inf_DP= {N_inf_DP_sol}\n' )


# Question 2: Batch solution of LQR problems
#============================================
print('Question 2: Batch solution of LQR problems')
# Write your code

N_batch_a_sol = 'The minimum horizon, when M=N, to give a stable closed-loop'
K_batch_a_sol = 'The feedback gain, when M=N'

print(f'The minimum N, when M=N, to give a stable closed-loop is \n N = {N_batch_a_sol}')
print(f'K =  \n {K_batch_a_sol} ')


# Write your code
 
N_batch_b_sol = 'The minimum horizon, when M<N, to give a stable closed-loop'
K_batch_b_sol = 'The feedback gain, when M<N'


print(f'\nThe minimum N, when M<N, to give a stable closed-loop is \n N = {N_batch_b_sol}')
print(f'K =  \n {K_batch_b_sol} ') 


# Question 3: Receding horizon control 
#=========================================
print('Question 3: Receding horizon control')
Nsim = 400; # number of simulation instances
x0 = np.array([-0.5,0,np.pi/12,0]) # initial state

# Write your code

# simulate the cart-pole plant (subplot for each state and control)

# Write your code


# Question 4: Constrained receding horizon control
print('\nQuestion 4: Constrained receding horizon control\n')

# simulate the plant (subplot for each state and control)