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
Using Some Rules:       sin(theta) = theta = x3
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
from sympy import simplify, cos, sin
x1, x2, x3, x4, u = sp.symbols('x1 x2 x3 x4 u')

D = 4/3*M - m*cos(x3)**2

dot_x1 = x2
dot_x2 = (4/3 * (u - m*l*x4**2 *sin(x3) + m*g*sin(x3)*cos(x3)))/D
dot_x3 =  x4
dot_x4 = (M*g*sin(x3) + cos(x3)*(u-m*l*x4**2 *sin(x3))) / (l*D)

x = sp.Matrix([x1,x2,x3,x4])
f = sp.Matrix([dot_x1,dot_x2,dot_x3,dot_x4])


"""
Now Apply Linearization around x = 0 and u = 0 
"""
Approximate = {cos(x3):1, sin(x3):x3}
f_approx =  f.subs(Approximate)

Ac = f_approx.jacobian(x)
Bc = f_approx.jacobian(sp.Matrix([u]))

Linearize = {x1:0, x2:0, x3:0, x4:0, u:0}

Ac0 = np.array(Ac.subs(Linearize), dtype=float)
Bc0 = np.array(Bc.subs(Linearize), dtype=float)

C = np.array([
    [1,0,0,0],
    [0,0,1,0]
])

A_c0_eigenvalue = np.linalg.eigvals(Ac0)

print(A_c0_eigenvalue)
Ac_sol = Ac0
Bc_sol = Bc0
Cc_sol = C

print('Continuous-time system')
print('Ac = \n',Ac_sol,'\n\n Bc = \n', Bc_sol,'\n\n Cc = \n', Cc_sol) 


# b) Discretizing the system 
#===========================

print('Question 1b)')

"""
Sampling Interval : h=0.1s
Find Matrices : A, B, C <- We can use Lemma 3 page 25
"""
h=0.1

Continuous_sub_Matrix = np.hstack((Ac0,Bc0))
#print(Continuous_sub_Matrix.shape)
Vector = np.zeros((1,5))
Continuous_Matrix = np.vstack((Continuous_sub_Matrix,Vector))
Discrete_Matrix = expm(Continuous_Matrix*h)

#print(Discrete_Matrix)
#print(Discrete_Matrix.shape)
#print("A:", Ac0.shape, "B",Bc0.shape)

A = Discrete_Matrix[0:4,0:4]
B = Discrete_Matrix[0:4,4:5]
A_eigenvalue = np.linalg.eigvals(A)

# write your code here 
A_1_sol = A
B_1_sol = B
C_1_sol = Cc_sol 
eig_A_1_sol = A_eigenvalue

print('Discrete-time system:\n')
print('A: \n',A_1_sol) 
print('\nB: \n',B_1_sol)
print('\nC: \n',C_1_sol)

# Inspect eigenvalues of the discrete system
print(f'\nEigenvalues of discrete-time system:\n')
print(eig_A_1_sol)


# c) Discrete model with delay
#=============================
"""
This case we have a delay tau = 0.8h
where h is the sample interval used previously
We need to find Aa, Ba, Ca, and Eig
"""
print('Question 1c)')
# write your code here 

tau  = 0.8*h

Delay_Matrix = expm(Continuous_Matrix*tau)
Atau = Delay_Matrix[0:4,0:4]
Btau = Delay_Matrix[0:4,4:5]

Discrete_Matrix_h = expm(Continuous_Matrix*(h-tau))
Ah = Discrete_Matrix_h[0:4,0:4]
Bh = Discrete_Matrix_h[0:4,4:5]


Aa_sol = np.vstack([
    np.hstack([Ah @ Atau, Ah @ Btau]),          
    np.hstack([np.zeros((1,4)), [[0]]]) # Bottom: u(k-1) becomes "old" input
])

Ba_sol = np.vstack([
    Bh, 
    [[1]]        
])
Ca_sol = np.hstack([C, np.zeros((2, 1))])
Aa_eigenvalue = np.linalg.eigvals(Aa_sol)
eig_Aa_sol= Aa_eigenvalue

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
K = -cl.dlqr(A,B,Q,R)[0]
print("K is:",K)
# Simulate the cart-pole plant 

x_0 = (np.array([-0.5, 0, np.pi/12, 0])).T

"""


#Copied From part 2
"""

T = 10
Nsim = int(T / h)
t = np.arange(0,Nsim,h)

# Plot the state feedback response

cp = CartPole(animate = True)    # instantiate the cart-pole simulator
cp.pauseTime= 0.01             # pause between plot frames
fig = cp.set_animation_figure()   # creates a figure (if you dont run this line, the simulator will Not produce a plot)
cp.x[:] = x_0 

# ----- simulation loop ----- 
Xsim = np.ones((Nsim,4))*np.inf
Usim = np.zeros(Nsim)
Xsim[0,:] = x_0
for k in range(Nsim):
    u = (K@cp.x).item()
    cp.simulate(F=u, dt=h, disturbance=0.0,title=f'Time step {k}/{Nsim}') # simulate one step
    Xsim[k] = cp.x # save state variables
    Usim[k] = u # Similar to the line above, where i save the control. :D
plt.show(block=True) # keep showing the last frame


"""
Here I am creating a plot/figure states and control trajectory
"""

fig2, axes = plt.subplots(5, 1, sharex=True, figsize=(6, 8))
t_plot = np.arange(Nsim) * h   # time vector for plotting

labels = ['$s$ (m)', r'$\dot{s}$ (m/s)', r'$\theta$ (rad)', r'$\dot{\theta}$ (rad/s)']
for i in range(4):
    axes[i].plot(t_plot, Xsim[:, i])
    axes[i].set_ylabel(labels[i])
    axes[i].grid(True)
axes[3].set_xlabel('Time (s)')

# Control
axes[4].plot(t_plot, Usim)
axes[4].set_ylabel('$u$ (N)')
axes[4].set_xlabel('Time (s)')
axes[4].grid(True)

plt.tight_layout()
plt.savefig('A1_Q3_trajectories.pdf', bbox_inches='tight')   # or .png for report
plt.show()

"""
# Question 4 : Steady-state targets
#==================================================
print('\nQuestion 4: Steady-state targets\n')


Here we investigate different setpoints for the output, the cart position and pole angle.

LQR is designed to steer the system to the origin.

The goal is perform a variable changes:
delta x = x-xs
delta u = u-us
Where xs and us are steady state targets. So that new delta values steer the target to the origin

Here we are to calculate state and input steady state targets, correspoing to output set point, for different cases.
Can ysp be tracked
"""
# case 1

A_matrix_case_1 = np.vstack([np.hstack([np.eye(4)-A, -B]), np.hstack([C, np.zeros((2,1))])])
print('Shape of A Matrix - Case 1:',A_matrix_case_1.shape) # 6 x 5. M>N. Thus Overdetermined System

#hint cond
condition_case_1 = np.linalg.cond(A_matrix_case_1)
print('Condition of A Matrix - Case 1',condition_case_1)


ysp_1 = np.array([[-1, np.pi/12]]).T
B_matrix_case_1 = np.vstack([np.zeros((4,1)),ysp_1])

x_case1 = np.linalg.lstsq(A_matrix_case_1,B_matrix_case_1)
ws_case1 = x_case1[0]
res_1 = x_case1[1]

ws_1_sol = ws_case1
print('For case 1, since the shape is 6x5 Overdetermined system: We check residuals')
print("--------------")
print("Residuals needs to be > 1e-10")
print("Res is: ",res_1)
print("Thes Steady-State Values:", ws_1_sol)
# case 2
C_case_2 = C[1:2,0:4]
A_matrix_case_2 = np.vstack([np.hstack([np.eye(4)-A, -B]), np.hstack([C_case_2, np.zeros((1,1))])])
print('Shape of A Matrix - Case 2:',A_matrix_case_2.shape)  # 5x5, Here we find exact solutions

#hint cond
condition_case_2 = np.linalg.cond(A_matrix_case_2)
print('Condition of A Matrix - Case 2',condition_case_2)

ysp_2 = np.array([[np.pi/12]]).T
B_matrix_case_2 = np.vstack([np.zeros((4,1)),ysp_2])

x_case2 = np.linalg.lstsq(A_matrix_case_2,B_matrix_case_2)

ws_case2 = x_case2[0]
rank_2 = x_case2[2]

ws_2_sol = ws_case2

print('For case 2, since the shape is 5x5 system: with inf solutions, we check rank')
print("--------------")
print("rank needs to be <5")
print("rank_2 is: ",rank_2)
print("Thes Steady-State Values:", ws_2_sol)

# case 3
C_case_3 = C[0:1,0:4]
A_matrix_case_3 = np.vstack([np.hstack([np.eye(4)-A, -B]), np.hstack([C_case_3, np.zeros((1,1))])])
print('Shape of A Matrix - Case 3:',A_matrix_case_3.shape)  # 5x5, Here we find exact solutions

#hint cond
condition_case_3 = np.linalg.cond(A_matrix_case_3)
print('Condition of A Matrix - Case 3',condition_case_3)

ysp_3 = np.array([[-1]]).T
B_matrix_case_3 = np.vstack([np.zeros((4,1)),ysp_3])

x_case3 = np.linalg.lstsq(A_matrix_case_3,B_matrix_case_3)

ws_case3 = x_case3[0]
rank_3 = x_case3[2]

ws_3_sol = ws_case3

print('For case 3, since the shape is 5x5 system: with a solutions, we check rank')
print("--------------")
print("rank needs to be =5")
print("rank_3 is: ",rank_3)
print("Thes Steady-State Values:", ws_3_sol)

# Question 5: Set-point tracking
#==================================================
print('\nQuestion 5: Set-point tracking\n')

# Use the steady-state targets from Case 3
xs = ws_case3[:4].reshape(-1, 1)  # Target state
us = ws_case3[4:].item()          # Target input

print(f'Setpoint: s = -1 m')
print(f'Steady-state targets:')
print(f'  xs = [{xs[0,0]:.4f}, {xs[1,0]:.4f}, {xs[2,0]:.4f}, {xs[3,0]:.4f}]^T')
print(f'  us = {us:.4f}')

# simulate the plant
T = 10
Nsim = int(T / h)
t_plot = np.arange(Nsim) * h

cp_q5 = CartPole(animate=True)
cp_q5.pauseTime = 0.01
fig_q5 = cp_q5.set_animation_figure()

x_0 = np.array([-0.5, 0, np.pi/12, 0])
cp_q5.x[:] = x_0

Xsim_q5 = np.ones((Nsim, 4)) * np.inf
Usim_q5 = np.zeros(Nsim)
Xsim_q5[0, :] = x_0

for k in range(Nsim):
    delta_x = (cp_q5.x - xs.flatten()).reshape(-1, 1)
    delta_u = (K @ delta_x).item()
    u = us + delta_u
    
    cp_q5.simulate(F=u, dt=h, disturbance=0.0, 
                   title=f'Q5: Setpoint Tracking - Step {k}/{Nsim}')
    
    Xsim_q5[k] = cp_q5.x
    Usim_q5[k] = u

plt.show(block=True)

# Plot the state feedback response

fig_q5_plots, axes = plt.subplots(5, 1, sharex=True, figsize=(8, 10))

labels = ['$s$ (m)', r'$\dot{s}$ (m/s)', r'$\theta$ (rad)', r'$\dot{\theta}$ (rad/s)']
for i in range(4):
    axes[i].plot(t_plot, Xsim_q5[:, i], label='Actual', linewidth=1.5)
    axes[i].axhline(xs[i].item(), color='r', linestyle='--', 
                    label=f'Target = {xs[i].item():.3f}', linewidth=1.5)
    axes[i].set_ylabel(labels[i])
    axes[i].legend(loc='best')
    axes[i].grid(True, alpha=0.3)

# Control
axes[4].plot(t_plot, Usim_q5, label='Actual', linewidth=1.5)
axes[4].axhline(us, color='r', linestyle='--', 
                label=f'Target = {us:.3f}', linewidth=1.5)
axes[4].set_ylabel('$u$ (N)')
axes[4].set_xlabel('Time (s)')
axes[4].legend(loc='best')
axes[4].grid(True, alpha=0.3)

plt.suptitle('Question 5: Setpoint Tracking to s = -1 m', fontsize=12, fontweight='bold')
plt.tight_layout()
plt.savefig('A1_Q5_setpoint_tracking.pdf', bbox_inches='tight')
plt.show()
