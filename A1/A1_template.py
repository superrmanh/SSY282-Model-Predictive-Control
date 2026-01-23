#
import numpy as np 
from numpy.linalg import norm 
import matplotlib.pyplot as plt 
from scipy.linalg import eig ,expm
import sys
import control as cl # control software
# add temporary path to the helper_funcs folder
sys.path.append("../helper_funcs")  # you might need to adjust the path
from cartpole import CartPole

# Question 1 : Linearization and discretization of a state-space model
#=====================================================================
print('Question 1: Linearization and discretization of a state-space model')
# a) Linearization
#==================
print('Question 1a)')

# System's Constants
g = 9.81   # [m/s^2] gravity
m = 0.1    # [kg] mass of the pole
M = 1.1    # [kg] total mass of pole and cart
l = 0.5    # [m] distance from the pole-cart attachment to the pole's center of mass

# Write your code here ...

"""
Nonlinear System : dot x(t) = f(x(t), u(t))
                       x(t) = [s(t), dot s(t), theta(t), dot theta(t)].T
                       u(t) = F(t)
Using Some Rules:       sin(theta) = theta 
                    cos(theta) = 1


First Approximate the nonlinear system, and linearize about x=0, u=0

Continuous System: dot x(t) = A_c x(t) + B_c u(t) 
                       y(t) = C_c x(t) 
Output Vector is: y(t) = [s(t) theta(t)].T
"""
# Equation We Were Given: 
# ddot_s(t) = (4/3( F(t) - ml dot_theta(t)^2 sin theta(t) + mgsin theta(t) cos theta(t))) /( 4/3M - mcos^2 theta(t))
# ddot_theta(t) = (Mgsin theta(t) + cos theta(t) (F(t) -ml dot_theta(t)^2 sin theta(t)))/(l(4/3M-mcos^2 theta(t)))
# But Lets First Convert All Of The Values into x_i

#from math import cos, sin
#from numpy import cos, sin
import sympy as sp
from sympy import cos, sin, simplify
x1, x2, x3, x4, u = sp.symbols('x1 x2 x3 x4 u')

D = 4/3*M - m*cos(x3)**2

dot_x1 = x2
dot_x2 = (4/3 * (u - m*l*x4**2 *sin(x3) + m*g*sin(x3)*cos(x3)))/D
dot_x3 =  x4
dot_x4 = (M*g*sin(x3) + cos(x3)*(u-m*l*x4**2 *sin(x3))) / (l*D)

x = sp.Matrix([x1,x2,x3,x4])
f = sp.Matrix([dot_x1,dot_x2,dot_x3,dot_x4])

Ac = (f.jacobian(x))
Bc = (f.jacobian(sp.Matrix([u])))


"""
Now Apply Linearization around x = 0 and u = 0 
"""
eq = {x1:0, x2:0, x3:0, x4:0}
Ac0 = Ac.subs(eq)
Bc0 = Bc.subs(eq)

Ac_sol = 'Matrix A for  time model'
Bc_sol = 'Matrix B for continuous time model'
Cc_sol = 'Matrix C for continuous time model'




print('Continuous-time system')
print('Ac = \n',Ac_sol,'\n\n Bc = \n', Bc_sol,'\n\n Cc = \n', Cc_sol) 

"""
# b) Discretizing the system 
#===========================

print('Question 1b)')
# write your code here 
A_1_sol = 'Matrix for the linearized, discrete-time system'
B_1_sol = 'Matrix for the linearized, discrete-time system' 
C_1_sol = 'Matrix for the linearized, discrete-time system' 
eig_A_1_sol = 'Eigenvalues of discrete-time system'

print('Discrete-time system:\n')
print('A: \n',A_1_sol) 
print('\nB: \n',B_1_sol)
print('\nC: \n',C_1_sol)

# Inspect eigenvalues of the discrete system
print(f'\nEigenvalues of discrete-time system:\n')
print(eig_A_1_sol)


# c) Discrete model with delay
#=============================

print('Question 1c)')
# write your code here 

Aa_sol = 'Augmented matrix for the discrete-time system with delay'
Ba_sol = 'Augmented matrix for the discrete-time system with delay'
Ca_sol = 'Augmented matrix for the discrete-time system with delay'
eig_Aa_sol= 'Eigenvalues of augmented discrete-time system with delay'

print('Augmented discrete-time system with delay; Aa, Ba, Ca:')
print('\n Aa = \n',Aa_sol) 
print('\n Ba = \n',Ba_sol)
print('\n Ca = \n',Ca_sol)
print('\nEigenvalues of augmented discrete-time system with delay:\n')
print(eig_Aa_sol)


# Question 2: Getting familiar with the cart-pole simulator
#==========================================================
print('Question 2: Getting familiar with the cart-pole simulator')

# ----- Initiate the CartPole object -----
cp = CartPole(animate = True)    # instantiate the cart-pole simulator
cp.pauseTime= 0.01             # pause between plot frames
fig = cp.set_animation_figure()   # creates a figure (if you dont run this line, the simulator will Not produce a plot)


x0 = np.array([0.0, 0.0, 0.0, 0.0] )    # initial state
cp.x[:] = x0  

# ----- Simulation paramters -----
dt = 0.025          # sampling time
T = 20              # [s] total simulation time
t = np.arange(0,T,dt)   # [s] time vector
Nsim = t.shape[0]

# ----- test control -----
Usim = 0.2*np.ones(Nsim) # push to the right for some time
Usim[144:464 ] = -0.2       # then push to the left

# ----- simulation loop ----- 
Xsim = np.ones((Nsim,4))*np.inf
Xsim[0,:] = x0
for k in range(Nsim):
    cp.simulate(F=Usim[k], dt=dt, disturbance=0.0,title=f'Time step {k}/{Nsim}') # simulate one step
    Xsim[k] = cp.x # save state variables
plt.show(block=True) # keep showing the last frame

# Question 3 : Linear-quadratic control
#=====================================================================
print('Question 3: Linear-quadratic control')

h = 0.025 # step size

# system matrices A,B C
A = np.array([
    [1.0000,    0.0250,    0.0002,    0.0000],
    [ 0,     1.0000,    0.0180,    0.0002],
    [ 0,         0,     1.0049,    0.0250],
    [ 0,         0,     0.3954,    1.0049]])

B =np.array([0.0003,    0.0244,    0.0005,    0.0366]).reshape(-1,1)
Q = np.eye(4)
R = 1

# feedback gain, hint: use cl.dlqr()
K = 'The optimal LQR gain'

# Simulate the cart-pole plant 


# Plot the state feedback response


# Question 4 : Steady-state targets
#==================================================
print('\nQuestion 4: Steady-state targets\n')

# case 1
ws_1_sol = 'Case 1 steady-state targets [xs; us], if any'

# case 2
ws_2_sol = 'Case 2 steady-state targets [xs; us], if any'

# case 3
ws_3_sol = 'Case 3 steady-state targets [xs; us], if any'

# Question 5: Set-point tracking
#==================================================
print('\nQuestion 5: Set-point tracking\n')

# simulate the plant

# Plot the state feedback response
"""