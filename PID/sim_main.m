%% File: sim_main.m
clear; close all; clc;

% load parameters
params = quad_params();

% simulation settings
Tsim = 25;           % total sim time [s]
dt   = 0.005;        % time step [s]
time = 0:dt:Tsim;

traj = @(t) helix_trajectory(t);
% traj = @(t) waypoint_trajectory(t);

[p0, ~, ~, yaw0] = traj(0);
x = zeros(12,1);
x(1:3)   = p0;
x(4:6)   = [0;0;0]; 
x(7:9)   = [0;0;yaw0]; 
x(10:12) = [0;0;0];

% controller state
ctrl_state.pos_int      = zeros(3,1);
ctrl_state.att_int      = zeros(3,1);
ctrl_state.phi_des_prev   = 0;
ctrl_state.theta_des_prev = 0;
ctrl_state.yaw_des_prev   = yaw0;

N = length(time);
X = zeros(12,N);
Urot = zeros(4,N);
Ref = zeros(6,N);
cost = zeros(1,N);

for k=1:N
    t = time(k);
    [p_ref, v_ref, a_ref, yaw_ref] = traj(t);
    Ref(:,k) = [p_ref; v_ref];

    [f_rotors, ctrl_state, ctrl_info] = controller_pid(x, p_ref, v_ref, a_ref, yaw_ref, ctrl_state, params, dt);

    Urot(:,k) = f_rotors;
    cost(k) = norm(p_ref - x(1:3));

    % dynamics step with RK4 (state derivatives from quad_dynamics)
    u_input.f = f_rotors; u_input.yaw_ref = yaw_ref;
    k1 = quad_dynamics(x, u_input, params);
    k2 = quad_dynamics(x + 0.5*dt*k1, u_input, params);
    k3 = quad_dynamics(x + 0.5*dt*k2, u_input, params);
    k4 = quad_dynamics(x + dt*k3, u_input, params);
    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

    X(:,k) = x;
end

figure('Name','Trajectory (3D)'); hold on; grid on; axis equal;
plot3(Ref(1,:), Ref(2,:), Ref(3,:), '--', 'LineWidth',1.2);
plot3(X(1,:), X(2,:), X(3,:), 'LineWidth',1.4);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('Reference','Actual');
title('3D trajectory tracking');

figure('Name','Position tracking errors');
subplot(3,1,1); plot(time, Ref(1,:)-X(1,:)); ylabel('ex [m]'); grid on;
subplot(3,1,2); plot(time, Ref(2,:)-X(2,:)); ylabel('ey [m]'); grid on;
subplot(3,1,3); plot(time, Ref(3,:)-X(3,:)); ylabel('ez [m]'); grid on; xlabel('Time [s]');

figure('Name','Rotor thrusts'); plot(time, Urot'); legend('f1','f2','f3','f4'); ylabel('Thrust [N]'); xlabel('Time [s]'); grid on;

animate_quadcopter(X, params, time, Ref);

fprintf('Final position error (norm): %.3f m\n', norm(Ref(1:3,end)-X(1:3,end)));

