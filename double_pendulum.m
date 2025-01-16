clear; close all; clc;
% Symbolic variables for system
syms theta0 theta1 theta2 dtheta0 dtheta1 dtheta2 ddtheta0 ddtheta1 ddtheta2
syms m0 m1 m2 l1 l2 L1 I1 I2 g u cr c1 c2

% State variables
syms x1 x2 x3 x4 x5 x6 % Define state variables explicitly

% Define generalized coordinates and their derivatives
theta = [theta0; theta1; theta2];
dtheta = [dtheta0; dtheta1; dtheta2];
ddtheta = [ddtheta0; ddtheta1; ddtheta2];

d1 = m0+m1+m2;
d2 = m1*l1+m2*L1;
d3 = m2*l2;
d4 = m1*l1^2+m2*L1^2+I1;
d5 = m2*L1*l2;
d6 = m2*l2^2+I2;
f1 = (m1*l1+m2*L1)*g;
f2 = m2*l2*g;

% Define the inertia matrix D(theta)
D = [d1, d2*cos(theta1), d3*cos(theta2);
     d2*cos(theta1), d4, d5*cos(theta1-theta2);
     d3*cos(theta2), d5*cos(theta1-theta2), d6];

% Define Coriolis and centrifugal terms C(theta, dtheta)
C = [cr, -d2*sin(theta1)*dtheta1, -d3*sin(theta2)*dtheta2;
     0, c1+c2, d5*sin(theta1-theta2)*dtheta2-c2;
     0, -d5*sin(theta1-theta2)*dtheta1-c2, c2];

% Define gravitational forces G(theta)
G = [0;
    -f1*sin(theta1);
    -f2*sin(theta2)];

% Define control input matrix b(theta)
H = [1;
     0;
     0];

% Euler-Lagrange equations
EL_eqns = D*ddtheta+C*dtheta+G-H*u == 0;

% Solve for accelerations (ddtheta)
ddtheta_sol = solve(EL_eqns, ddtheta);

% Simplify each solution
ddtheta0_sol = simplify(ddtheta_sol.ddtheta0);
ddtheta1_sol = simplify(ddtheta_sol.ddtheta1);
ddtheta2_sol = simplify(ddtheta_sol.ddtheta2);

% State-space representation
% Define state vector x = [theta0, dtheta0, theta1, dtheta1, theta2, dtheta2]
states = [x1; x2; x3; x4; x5; x6];

% Substitute theta and dtheta with state variables
dtheta_subs = [x2; x4; x6];
dx = [x2;
      ddtheta0_sol;
      x4;
      ddtheta1_sol;
      x6;
      ddtheta2_sol];

% Substitute symbolic expressions into dx
dx = subs(dx, [theta; dtheta], [x1; x3; x5; dtheta_subs]);

% Display state-space equations
%disp('State-space equations (dx):');
%disp(dx);

% Linearization around equilibrium point (all zeros)
A = jacobian(dx, states); % State matrix
B = jacobian(dx, u);      % Input matrix

%% Linearized state-space model
% Substitute equilibrium point values (all states = 0)
A_lin = simplify(subs(A, states, zeros(size(states))));
B_lin = simplify(subs(B, states, zeros(size(states))));

% Display linearized matrices
%disp('A matrix:');
%disp(A_lin);
%disp('Eigh values')
%disp(eig(A_lin))
%disp('B matrix:');
%disp(B_lin);

%% Inverted Pendulum Model Parameters

params = [m0, m1, m2, I1, I2, g, l1, l2, L1, cr, c1, c2];
values = [2.44, 0.308, 0.253, 0.0036, 0.0024, 9.8, 0.181, 0.153, 0.317, 19, 0.02, 0.02];

% Substitute into A and B
A_num = double(subs(A_lin, params, values));
B_num = double(subs(B_lin, params, values));

l1 = 0.3;
l2 = 0.3;

disp('A matrix:');
disp(A_lin)
disp(A_num);

disp('B matrix:');
disp(B_lin)
disp(B_num);

C_num = [1 0 0 0 0 0;
         0 0 1 0 0 0;
         0 0 0 0 1 0];
            
D_num = [0;
         0;
         0];


%% Discretize the system
Ts = 0.1; % Sampling time [s]
sys = ss(A_num, B_num, C_num, D_num);
sys_d = c2d(sys, Ts, 'zoh'); % Discretized system
Ad = sys_d.a;
Bd = sys_d.b;
Cd = sys_d.c;


%% Design MPC Controller
N = 20; % Prediction horizon
Nu = 5; % Control horizon
Q = diag([10, 1, 100, 1, 0, 0]); % State weighting matrix
R = 0.1; % Control input weighting
% Create MPC controller object
mpcobj = mpc(sys_d, Ts);
mpcobj.PredictionHorizon = N;
mpcobj.ControlHorizon = Nu;
mpcobj.Weights.OutputVariables = [10, 100, 0];
mpcobj.Weights.ManipulatedVariables = R;
mpcobj.Weights.ManipulatedVariablesRate = 0.1;
% Constraints
mpcobj.MV(1).Min = -10; % Force min [N]
mpcobj.MV(1).Max = 10; % Force max [N]


%% Simulation
Nsim = 100; % Number of simulation steps
x0 = [0.2; 0; -0.1; 0; 0.1; 0]; % Initial state

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
%mpcstate = mpcstate(mpcobj, x(:,1));
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


%% Plot the results
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
    
    pendulum_x1 = [y(1,k), y(1,k) + l1 * sin(y(2,k))];
    pendulum_y1 = [0, -l1 * cos(y(2,k))];

    pendulum_x2 = [y(1,k) + l1 * sin(y(2,k)), y(1,k) + l1 * sin(y(2,k)) + l2 * sin(y(3,k))];
    pendulum_y2 = [-l1 * cos(y(2,k)), -l1 * cos(y(2,k)) - l2 * cos(y(3,k))];
    
    % Draw cart
    rectangle('Position', [cart_x, cart_y, cart_width, cart_height], 'FaceColor', 'b');
    
    % Draw pendulum 1
    plot(pendulum_x1, -pendulum_y1, 'r', 'LineWidth', 2);
    plot(pendulum_x1(2), -pendulum_y1(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

    % Draw pendulum 2
    plot(pendulum_x2, -pendulum_y2, 'g', 'LineWidth', 2);
    plot(pendulum_x2(2), -pendulum_y2(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    
    % Draw rail
    line([-1, 1], [-cart_height/2, -cart_height/2], 'Color', 'k', 'LineWidth', 1);
    
    % Set axis limits and labels
    xlim([-1, 1]);
    ylim([-1, 1]);
    xlabel('Position (m)');
    ylabel('Height (m)');
    title('Double Inverted Pendulum Animation');
    
    drawnow;
    pause(0.05);
end