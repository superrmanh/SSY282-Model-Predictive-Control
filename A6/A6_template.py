import numpy as np 
import sys
# add path to the helper_funcs folder
sys.path.append("../helper_funcs")  # you might need to adjust the path
from cartpole import CartPole
# Question 1: Lyapunov functions
#=============================================
print('\nQuestion 1: Lyapunov functions\n')

A = np.array([
    [1.0041, 0.01, 0, 0],
    [0.8281, 1.0041, 0, -0.0093],
    [0.0002, 0, 1, 0.0098],
    [0.0491, 0.0002, 0, 0.9629]
    ])
B = np.array([0.0007,0.1398,0.0028,0.5605]).reshape(-1,1)

eig_Qa_sol = 'Eigenvalues of Q'
eig_A_sol = 'Eigenvalues of A'

# b)
K = np.array([-114.3879, -12.7189, 1.2779, 1.5952])

eig_Qb_sol = 'Eigenvalues of Q'
S_sol = 'Lyapunov matrix'
eig_S_sol = 'Eigenvalues of S'

# Question 2
#==========================================
print('\nQuestion 2: Stability with receding horizon control\n')
Q = np.eye(4)
R = 1

# a)
N = 1

# b)
Pf = Q

N_sol = 'The minimum horizon to give a stable closed-loop system'
K_b_sol = 'The feedback gain'

# c)

# d)
Pf_sol = 'Final cost for which the system is stable'
K_d_sol = 'The feedback gain'

# Question 3
#==========================================
print('\nQuestion 3: Stabilization of an unstable system\n')

# Question 4
print('\nQuestion 4: Asymptotic stability for constrained systems\n')
h = 0.025 # sampling interval 
A = np.array([
        [1.0000,    0.0250,    0.0002,    0.0000],
        [0,     1.0000,    0.0180,    0.0002],
        [0,         0,     1.0049,    0.0250],
        [0,         0,     0.3954,    1.0049]])
B = np.array([0.0003,    0.0244,    0.0005,    0.0366]).reshape(-1,1)
Q = np.eye(4)
Pf = 10*np.eye(4)
R = 1
xmin = -np.inf*np.ones(4)
xmax = -xmin
umin = -5
umax = 5
Nsim = 400 # number of simulation instances
x0 = np.array([-0.5, 0, np.pi/12, 0]) # initial state


plant = CartPole(animate=True)
fig = plant.set_animation_figure() 
plant.pauseTime= 0.005  # [s] pause time between animation frames
plant.x = x0

# a) 
N = 7
Ff = np.array([
    [1,     1,     0,     0],
     [-1,    -1,     0,     0],
     [ 0,     0,     1,     0.2],
     [ 0,     0,    -1,    -0.2]])
hf = np.array([1.5, 1.5, 0.1, 0.1]).reshape(-1,1)

# b) 
N = 1
Pf4_sol = 'Final cost that keeps the system is stable'