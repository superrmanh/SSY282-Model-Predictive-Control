import numpy as np 
import matplotlib.pyplot as plt 
from scipy.linalg import block_diag
import sys
# add path to the helper_funcs folder
sys.path.append("../helper_funcs")  # you might need to adjust the path
from cartpole import CartPole

# The cart-pole simulator under disturbance
#===============================================
A = np.array([
    [1.0000,    0.0250,    0.0002,    0.0000],
    [ 0,     1.0000,    0.0180,    0.0002],
    [ 0,         0,     1.0049,    0.0250],
    [ 0,         0,     0.3954,    1.0049]])

B =np.array([0.0003,    0.0244,    0.0005,    0.0366]).reshape(-1,1)


C = np.eye(4); ny = C.shape[0]

Q = np.eye(4)
R = 0.1
Pf = 10*np.eye(4)
nx, nu = B.shape  # number of states and controls
N= 80

# start the cartpole plant
plant = CartPole(animate=True)    # instantiate the cart-pole simulator
fig = plant.set_animation_figure() 
Nsim = 400
h = 0.025
x0 = np.array([0,0,0,0])
plant.x[:] = x0  
Fd = 0.1 

X = np.ones((Nsim+1,nx))
U = np.zeros((Nsim,nu))

x = x0
for k in range(Nsim):
    U[k] = -Fd # TODO: Replace this the control with your own controller to converge to a steady-state solution
    X[k+1,:] = plant.simulate(U[k,:].item(),dt= h,disturbance= Fd,title=f'Cart-Pole RHC, Fd = {Fd}')
plt.close(fig)


# Question 1
#========================================
print('\nQuestion 1: Detectability of the augmented model\n')
# write your code
detectable_sol = 'Booleand vector of four elements, with true if detectable'

# Question 2
#========================================
print('\nQuestion 2: Steady-state targets and a Kalman filter\n')
# write your code

Mss_sol = 'Matrix for computing steady-state targets'

L_sol = 'Kalman gain'

# Question 3
#========================================
print('\nQuestion 3: Receding horizon control\n')
# write your code

# simulate the plant
# write your code
