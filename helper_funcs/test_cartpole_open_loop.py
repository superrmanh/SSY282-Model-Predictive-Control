from cartpole import CartPole
import numpy as np
import matplotlib.pyplot as plt

cp = CartPole(animate=True) # instantiate the cart-pole simulator
cp.pauseTime= 0.01          # pause between plot frames
cp.set_animation_figure()   # creates a figure (if you dont run this line, the simulator will Not produce a plot)
x0 = np.array([0.0, 0.0, 0.0, 0.0] )    # initial state
cp.x[:] = x0  
dt = 0.025          # sampling time
T = 20              # simulation time
t = np.arange(0,T,dt)   # [s] time vector
Nsim = t.shape[0]

# test control
Usim = 0.2*np.ones(Nsim) # push to the right for some time
Usim[144:464 ] = -0.2       # then push to the left

# simulation loop
Xsim = np.ones((Nsim,4))*np.inf
Xsim[0,:] = x0
for k in range(Nsim):
    cp.simulate(F=Usim[k], dt=dt, disturbance=0.0,title=f'Time step {k}/{Nsim}')
    Xsim[k] = cp.x

plt.figure()
plt.plot(t,Xsim[:,3],'r-')
plt.grid()
plt.xlabel('Time [s]')
plt.ylabel(f'$d\\theta/dt$')
plt.show()


# if you use Jupyter notebook make sure to pass notebook_mode = True in set_animation_figure
