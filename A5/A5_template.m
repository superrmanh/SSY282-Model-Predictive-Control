clear all; clc
addpath('../helper_funcs');

%% Install MPT (uncomment and run the 3 lines below)
% MPT_install;
% mpt_demo_sets1;
% mpt_demo2;

%% Question 1
fprintf('Question 1: Feasibility\n');

A = [0.9 0.4
     -0.4 0.9];
B = [0; 1];
xmin =-[3; 3];
xmax = [3; 3];
umin =-0.1;
umax = 0.1;
x0 = [-1; 0];

% a) shortest prediction horizon N such that the RH controller is feasible
Xf = Polyhedron([0 0]);

N_sol = 'Shortest horizon for a feasible controller';

% b) maximal control invariant
N = 2;
feasible_sol = 'Feasible controller when Xf=Cinf?'; % true or false
N_vars_a_sol = 'Number of variables, case (a)'; 
N_vars_b_sol = 'Number of variables, case (b)'; 
N_cstr_a_sol = 'Number of constraints, case (a)'; 
N_cstr_b_sol = 'Number of constraints, case (b)';

%% Question 2
fprintf('\nQuestion 2: Minimum time control and explicit MPC\n');

A = [1.2, 1
     0, 1];
B = [0; 1];
xmin =-[15; 15];
xmax = -xmin;
umin =-1;
umax = 1;

% a) find Pf to guarantee asymptotic stability for all x0
Q = eye(2);
R = 100;
N = 4;
Xf = Polyhedron([0 0]); % target states set

Pf_sol = 'Pf to guarantee asymptotic stability for all x0';

% b) Open-loop vs closed-loop control
x0 = [7; -4];
N_val = [10, 15, 20];

% c) Minimum-time control
N_min_sol = 'The minimim number of samples to reach Xf';

% d) Explicit MPC
N = 20;
Pf = 10*Q;
Xf = Polyhedron('lb',-[0.01; 0.01],'ub',[0.01; 0.01]);

%% Question 3
fprintf('\nQuestion 3: Implicit vs. explicit MPC\n');
h = 0.025; % sampling interval 
A = [1.0000    0.0250    0.0002    0.0000;
         0     1.0000    0.0180    0.0002;
         0         0     1.0049    0.0250;
         0         0     0.3954    1.0049];
B = [0.0003    0.0244    0.0005    0.0366]';
Q = eye(4);
R = 1;
Pf = [90          90        -300         -90;
      90         170        -700        -170;
     -300        -700        5200        1300;
     -90        -170        1300         340];
N = 2;
umin = -5; 
umax = 5;
Nsim = 400; % number of simulation instances
x0 = [-0.5; 0; pi/12; 0]; % initial state

comp_time_ratio_sol = '(Comp. time implicit)/(comp. time explicit)';
num_regions_sol = 'Number of partitions for N=1,...,5';
