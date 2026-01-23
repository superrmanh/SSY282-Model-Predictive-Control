clearvars;clc;close all;
addpath('../helper_funcs');

%% Question 1
fprintf('Question 1: Dynamic Programming solution of the LQ problem\n');

% a) Stabilizing controller through Dynamic Programming
h = 0.025; % sampling interval 
A = [1.0000    0.0250    0.0002    0.0000;
         0     1.0000    0.0180    0.0002;
         0         0     1.0049    0.0250;
         0         0     0.3954    1.0049];
B = [0.0003    0.0244    0.0005    0.0366]';
Q = eye(4);
Pf = 10*eye(4);
R = 1;

N_DP_sol = 'The minimum horizon to give a stable closed-loop';
K_DP_sol = 'The feedback gain'; 

% b) Stationary Riccati matrix
P_inf_dare_sol = 'The stationary Riccati matrix using dare';
P_inf_DP_sol = 'The stationary Riccati matrix from DP';
N_inf_DP_sol = 'The number of DP iterations until convergence';

%% Question 2
fprintf('\nQuestion 2: Batch solution of the LQ problem\n');

N_batch_a_sol = 'The minimum horizon, when M=N, to give a stable closed-loop';
K_batch_a_sol = 'The feedback gain, when M=N';

N_batch_b_sol = 'The minimum horizon, when M<N, to give a stable closed-loop';
K_batch_b_sol = 'The feedback gain, when M<N';

%% Question 3
fprintf('\nQuestion 3: Receding horizon control\n');

Nsim = 400; % number of simulation instances
x0 = [-0.5; 0; pi/12; 0]; % initial state
R_val = [1, 1, 0.1, 0.1];
N_val = [40, 80, 40, 80];

% simulate the plant (subplot for each state and control)

%% Question 4
fprintf('\nQuestion 4: Constrained receding horizon control\n');

% simulate the plant (subplot for each state and control)
