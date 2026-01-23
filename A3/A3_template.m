clearvars;clc;close all;
addpath('../helper_funcs');

%% The cart-pole simulator under disturbance
fprintf('\nThe cart-pole simulator under disturbance\n');
h = 0.025; % sampling interval 

A = [1.0000    0.0250    0.0002    0.0000;
         0     1.0000    0.0180    0.0002;
         0         0     1.0049    0.0250;
         0         0     0.3954    1.0049];

B = [0.0003    0.0244    0.0005    0.0366]';

C = eye(4);

plant = cartpole();
plant.animationFig = figure('WindowState', 'maximized');
plant.pauseTime = 0.005; % [s] pause time between animation frames
% plant.animate = false; close(plant.animationFig); % comment out this line to run the animation

% different controller settings
Q = eye(4);
Pf = 10*eye(4);
R = 0.1;
N = 80;

Nsim = 400; % number of simulation instances
x0 = [0; 0; 0; 0]; % initial state
Fd = 0.1;   % [N] disturbance force

% replace the control action below with your own controller to converge to
% a steady-state solution
u = -Fd; % dummy control (TO BE REPLACED)
plant.x = x0;
for t = 1:Nsim
    plant.simulate(u, h, 'disturbance', Fd);
end

%% Question 1
fprintf('\nQuestion 1: Detectability of the augmented model\n');

detectable_sol = 'Booleand vector of four elements, with true if detectable';

%% Question 2
fprintf('\nQuestion 2: Steady-state targets and a Kalman filter\n');

Mss_sol = 'Matrix for computing steady-state targets';

L_sol = 'Kalman gain';

%% Question 3
fprintf('\nQuestion 3: Receding horizon control\n');

% simulate the plant
