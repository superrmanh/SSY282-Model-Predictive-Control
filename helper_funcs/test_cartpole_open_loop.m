clearvars; clc;

plant = cartpole();
plant.animate = true;
plant.animationFig = figure('WindowState', 'maximized');
plant.pauseTime = 0.005; % [s] pause time between animation frames

h = 0.025;             % [s] sampling interval
t = 0:h:20;            % [s] time vector
Nsim = numel(t) - 1;   % number of time samples
x0 = [0; 0; 0; 0];    % [m; m/s; rad; rad/s] initial state 

% apply some control action F: during the first quarter of time F=0.2 N.
% Aftewards, F=-0.2 N;
Usim = 0.2*ones(1, Nsim);
Usim(round(0.18*Nsim):round(0.58*Nsim)) = -0.2;

Xsim = NaN(4, Nsim + 1);
Xsim(:, 1) = x0;
plant.x = x0;

for k=1:Nsim
    Xsim(:, k+1) = plant.simulate(Usim(:,k), h);
end

%% plot
figure; 
plot(t, Xsim(4,:)); grid on;
xlabel("Time (s)");
ylabel("d\theta/dt");
title("Open-Loop simulation of the cart-pole system");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2025-12.