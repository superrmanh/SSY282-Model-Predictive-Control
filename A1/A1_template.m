clearvars; clc; close all;
addpath('../helper_funcs'); % set path to helper_funcs, if needed

%% Question 1
fprintf('Question 1: Linearization and discretization of a state-space model\n');
% a) Linearized continuous-time model
g = 9.81;   % [m/s^2] gravity
m = 0.1;    % [kg] mass of the pole
M = 1.1;    % [kg] total mass of pole and cart
l = 0.5;    % [m] distance from the pole-cart attachment to the pole's center of mass

Ac_sol = 'Matrix for the linearized, continuous-time system';

Bc_sol = 'Matrix for the linearized, continuous-time system';

Cc_sol = 'Matrix for the linearized, continuous-time system';

eig_Ac_sol = 'Eigenvalues of continuous-time system';

% b) Discrete model
h = 0.1;  % [s] sampling interval

A_1_sol = 'Matrix for the linearized, discrete-time system';
B_1_sol = 'Matrix for the linearized, discrete-time system';
C_1_sol = 'Matrix for the linearized, discrete-time system';

eig_A_1_sol = 'Eigenvalues of discrete-time system';

% c) Discrete model with delay
tau = 0.8*h; %computational delay

Aa_sol = 'Augmented matrix for the discrete-time system with delay';
Ba_sol = 'Augmented matrix for the discrete-time system with delay';
Ca_sol = 'Augmented matrix for the discrete-time system with delay';

% compute eigenvalues/poles
eig_Aa_sol = 'Eigenvalues of augmented discrete-time system with delay';

%% Question 2
fprintf('\nQuestion 2: Getting familiar with the cart-pole simulator\n');

plant = cartpole();
plant.animate = true;
plant.animationFig = figure('WindowState', 'maximized');
plant.pauseTime = 0.005; % [s] pause time between animation frames
% plant.animate = false; close(plant.animationFig);

h = 0.025; % sampling interval 
plnat.x = zeros(4, 1); % initial state
for k=1:850 % number of time samples
    % test some control action u=F
    if k > 130 && k < 450
        u = -0.2; % [N]
    else
        u = 0.2; % [N]
    end
    x = plant.simulate(u, h);
end

%% Question 3
fprintf('\nQuestion 3: Linear-quadratic control\n');

h = 0.025; % sampling interval 

% system matrices A, B, C:
A = [1.0000    0.0250    0.0002    0.0000;
         0     1.0000    0.0180    0.0002;
         0         0     1.0049    0.0250;
         0         0     0.3954    1.0049];

B = [0.0003    0.0244    0.0005    0.0366]';

Q = diag([1,1,1,1]);
R = 1;

% feedback gain
K_sol = 'The optimal LQR gain'; 

% simulate the plant

% Plot the state feedback response

%% Question 4
fprintf('\nQuestion 4: Steady-state targets\n');

% case 1
ws_1_sol = 'Case 1 steady-state targets [xs; us], if any';

% case 2
ws_2_sol = 'Case 2 steady-state targets [xs; us], if any';

% case 3
ws_3_sol = 'Case 3 steady-state targets [xs; us], if any';

%% Question 5
fprintf('\nQuestion 5: Set-point tracking\n');

% simulate the plant

% Plot the state feedback response