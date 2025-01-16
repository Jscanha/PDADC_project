% (c) Yajur Kumar, 2023. All rights reserved.
% Program to simulate control of inverted pendulum using Model Predictive
% Control

clear; close all; clc;
%% Inverted Pendulum Model Parameters
M = .5; % Mass of the cart [kg]
m = 0.2;% Mass of the pendulum [kg]
b = 0.1;% Coefficient of friction for cart [N/m/s]
I = 0.006; % Maximum moment of inertia
g = 9.8; % Gravity [m/s^2]
l = 0.3; % Length to pendulum center of mass [m]


%% Linearized state-space model
p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0 1 0 0;
     0 -(I+m*l^2)*b/p (m^2*g*l^2)/p 0;
     0 0 0 1;
     0 -(m*l*b)/p m*g*l*(M+m)/p 0];
B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];
C = [1 0 0 0;
     0 0 1 0];
D = [0;
     0];

%% Discretize the system
Ts = 0.1; % Sampling time [s]
sys = ss(A, B, C, D);
sys_d = c2d(sys, Ts, 'zoh'); % Discretized system
Ad = sys_d.a;
Bd = sys_d.b;
Cd = sys_d.c;

%% Design MPC Controller


%% Simulation
Nsim = 100; % Number of simulation steps
x0 = [0.2; 0; pi/4; 0]; % Initial state
% Preallocate arrays for simulation results
nx = size(Ad, 1);
nu = size(Bd, 2);
ny = size(Cd, 1);
x = zeros(nx, Nsim);
y = zeros(ny, Nsim);
u = zeros(nu, Nsim);
x(:,1) = x0;
y(:,1) = Cd * x0;
% Create initial MPC state
mpcstate = mpcstate(mpcobj, x(:,1));
% Main simulation loop
for k = 1:Nsim-1
    % Measure the current output
    ymeas = y(:,k);
    
    % Compute the control action using the MPC controller
    %[u(:,k), mpc_info] = mpcmove(mpcobj, mpcstate, ymeas, []);
    %- AQUI METES A FUNÇÃO PARA DETERMINAR A AÇÃO DE CONTROLO
    
    % Apply the control action to the system and obtain the next state
    x(:,k+1) = Ad * x(:,k) + Bd * u(:,k);
    
    % Calculate the output
    y(:,k+1) = Cd * x(:,k+1);
    
    % Update MPC state
    %mpcstate.Plant = x(:,k+1);
    % AQUI METER O REWARD SYSTEM -> controlo para o Reenforcment learning
end
% Plot the results
t = (0:Nsim-1) * Ts;
figure(1);
subplot(3,1,1);
plot(t, y(1,:), 'LineWidth', 2);
ylabel('Cart Position (m)');
title('Inverted Pendulum MPC Control');
grid on;
subplot(3,1,2);
plot(t, y(2,:), 'LineWidth', 2);
ylabel('Pendulum Angle (rad)');
grid on;
subplot(3,1,3);
plot(t, u(:,1:Nsim), 'LineWidth', 2); % Corrected dimensions of u
xlabel('Time (s)');
ylabel('Force (N)');
grid on;

%% Animate Pendulum
figure(2);
xlim([-1, 1]);
ylim([-1, 1]);
hold on;
% Define cart and pendulum parameters
cart_width = 0.2;
cart_height = 0.1;
for k = 1:Nsim
    cla; % Clear current axes
    cart_x = y(1,k) - cart_width / 2;
    cart_y = -cart_height / 2;
    
    pendulum_x = [y(1,k), y(1,k) + l * sin(y(2,k))];
    pendulum_y = [0, -l * cos(y(2,k))];
    
    % Draw cart
    rectangle('Position', [cart_x, cart_y, cart_width, cart_height], 'FaceColor', 'b');
    
    % Draw pendulum
    plot(pendulum_x, -pendulum_y, 'r', 'LineWidth', 2);
    plot(pendulum_x(2), -pendulum_y(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    % Draw rail
    line([-1, 1], [-cart_height/2, -cart_height/2], 'Color', 'k', 'LineWidth', 1);
    
    % Set axis limits and labels
    xlim([-1, 1]);
    ylim([-1, 1]);
    xlabel('Position (m)');
    ylabel('Height (m)');
    title('Inverted Pendulum Animation');
    
    drawnow;
    pause(0.05);
end