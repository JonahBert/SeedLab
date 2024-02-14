%% motorControl.m
% This script runs a simulation of a motor and plots the results
%
% required file: motorsim.slx
%
%% Define motor parameters
Kp = 4;
KP = 30;
KI = 1.0;
K = 1.0; % DC gain [rad/Vs]
sigma=18; % time constant reciprocal [1/s]
%% Run a Simulation
%
% open the block diagram so it appears in the documentation when published.
% Make sure the block diagram is closed before running the publish function
%
open_system('PIController')
%
% run the simulation
%

out=sim('PIController');
%% A Plot of the results
%
figure
subplot(3,1,1)
plot(out.Voltage,'linewidth',1)
xlabel('Time (s)')
ylabel('Voltage (V)')
title('Voltage Control')
subplot(3,1,2)
plot(out.DesiredVelocity, 'linewidth', 1);
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title('Velocity Control')
subplot(3,1,3)
plot(out.DesiredPosition, 'linewidth', 1);
hold on
plot(out.Position, 'linewidth', 1);
legend('Desired Position', 'Actual Position')
title('Position Control')
xlabel('Time (s)')
ylabel('Position (rad)')
