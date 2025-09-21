% sim_main.m
clear; close all; clc;

dt = 0.005; Tsim = 25; time = 0:dt:Tsim; N = numel(time);

params = quad_params_maneuver_v1();
% traj   = @(t) helix_trajectory(t);
traj = @(t) waypoint_trajectory_smooth(t);

% Precompute initial reference & state
[p0, v0, a0, yaw0] = traj(0);
x = zeros(12,1);
x(1:3) = p0; x(4:6) = v0; x(7:9) = [0;0;yaw0]; x(10:12) = 0;

% Controller state 
ctrl.pos_int = zeros(3,1);
ctrl.pos_int_f = zeros(3,1);
ctrl.att_int = zeros(3,1);
ctrl.phi_des_prev = 0; ctrl.theta_des_prev = 0; ctrl.yaw_des_prev = yaw0;
ctrl.phi_des_f = 0; ctrl.theta_des_f = 0;
ctrl.tau_ang_cmd_filter = 0.010;
hover_total = params.m * params.g;
ctrl.f_prev = clamp((hover_total/4) * ones(4,1), params.f_min, params.f_max);
ctrl.U1_prev = hover_total;
ctrl.a_ref_f = a0;
ctrl.yaw_ref_prev = yaw0;
ctrl.yaw_ref_f = yaw0;

% Logging
X = zeros(12,N); Urot = zeros(4,N); Ref = zeros(6,N);
sat_flag_log = false(1,N); int_log = zeros(3,N); maneuver_log = false(1,N);

% Maneuver detection tuning 
a_lat_maneuver_thresh = 0.8;
yaw_rate_thresh = 4.5;
PD_scale_maneuver = 1.6;
Klead_scale_maneuver = 1.6;
Ki_scale_maneuver = 0.0;
att_rate_limits_maneuver = deg2rad([320;320;220]);
U1_rate_max_maneuver = 120.0;
pos_int_rate_limit_base = [0.12;0.12;0.20];
pos_int_freeze_thresh = 0.35;
sat_bleed = 0.04;
Klead_pos_base = diag([0.25,0.25,0.12]);
Ki_scale_base = 1.0;
int_filter_tau_base = 0.12;
D_omega_base = diag([0.008,0.008,0.004]);
tau_rot = 0.06;
U1_rate_max_base = 50.0;
f_rate_max = 1e6;
tau_a_ref_base = 0.6;
maneuver_exit_hold = 0.40;
tau_yaw_ref = 0.12;

entered_maneuver = false;
maneuver_start_time = -1;

I = params.I;
A_att = zeros(6,6); A_att(1,4)=1; A_att(2,5)=1; A_att(3,6)=1;
B_att = zeros(6,3); B_att(4:6,:) = I \ eye(3);
Q_att = diag([300,300,30,6,6,3]);
R_att = diag([0.18,0.18,0.12]);
try
    K_att = lqr(A_att, B_att, Q_att, R_att);
catch
    [P,~,~] = care(A_att, B_att, Q_att, R_att);
    K_att = (R_att \ (B_att' * P));
end
Ki_att = diag([0.45,0.45,0.12]);
att_int_lim = deg2rad([3;3;1.5]);
tau_max = [2.3;2.3;1.4];

%% Main sim loop
for k=1:N
    t = time(k);
    [p_ref, v_ref, a_ref_raw, yaw_ref_unfilt] = traj(t);
    Ref(:,k) = [p_ref; v_ref];

    alpha_yaw = dt / (tau_yaw_ref + dt);
    yaw_prev_f = ctrl.yaw_ref_f;
    dy = wrapAngle(yaw_ref_unfilt - yaw_prev_f);
    ctrl.yaw_ref_f = wrapAngle(yaw_prev_f + alpha_yaw * dy);
    yaw_rate_ref = wrapAngle(ctrl.yaw_ref_f - ctrl.yaw_ref_prev) / max(dt,1e-9);
    ctrl.yaw_ref_prev = ctrl.yaw_ref_f;
    yaw_ref = ctrl.yaw_ref_f;

    % accel LPF
    alpha_a = dt / (tau_a_ref_base + dt);
    ctrl.a_ref_f = (1-alpha_a) * ctrl.a_ref_f + alpha_a * a_ref_raw;
    a_ref = ctrl.a_ref_f;

    % detect maneuver by ref
    a_lat_ref = norm(a_ref(1:2));
    high_yaw = abs(yaw_rate_ref) > yaw_rate_thresh;
    high_lat = a_lat_ref > a_lat_maneuver_thresh;
    maneuver_flag = high_yaw || high_lat;

    if maneuver_flag && ~entered_maneuver
        entered_maneuver = true; maneuver_start_time = t;
        fprintf('Maneuver mode ENTER at t = %.3f  (a_lat_ref=%.3f m/s^2, yaw_rate=%.3f rad/s)\n', t, a_lat_ref, yaw_rate_ref);
    elseif ~maneuver_flag && entered_maneuver
        if t - maneuver_start_time > maneuver_exit_hold
            entered_maneuver = false;
            fprintf('Maneuver mode EXIT at t = %.3f\n', t);
        else
            maneuver_flag = true;
        end
    end
    maneuver_log(k) = maneuver_flag;

    if maneuver_flag
        PD_scale = PD_scale_maneuver;
        Klead_pos = Klead_pos_base * Klead_scale_maneuver;
        Ki_scale = Ki_scale_maneuver;
        att_rate_limits = att_rate_limits_maneuver;
        U1_rate_max = U1_rate_max_maneuver;
        pos_int_rate_limit = [0.02;0.02;0.02];
        integrator_freeze = true;
    else
        PD_scale = 1.0;
        Klead_pos = Klead_pos_base;
        Ki_scale = Ki_scale_base;
        att_rate_limits = params.att_rate_max;
        U1_rate_max = U1_rate_max_base;
        pos_int_rate_limit = pos_int_rate_limit_base;
        integrator_freeze = false;
    end

    [f_rotors, ctrl, sat_flag, sat_heavy] = controller_maneuver_adaptive(...
        x, p_ref, v_ref, a_ref, yaw_ref, ctrl, params, ...
        dt, K_att, Ki_att, att_int_lim, tau_max, ...
        0.28, pos_int_freeze_thresh, pos_int_rate_limit, sat_bleed, ...
        U1_rate_max, f_rate_max, att_rate_limits, tau_rot, D_omega_base, ...
        Klead_pos, Ki_scale, int_filter_tau_base, PD_scale, integrator_freeze);

    Urot(:,k) = f_rotors;
    sat_flag_log(k) = sat_flag;
    int_log(:,k) = ctrl.pos_int_f;

    % RK4 integrate
    u_input.f = f_rotors; u_input.yaw_ref = yaw_ref;
    k1 = quad_dynamics(x, u_input, params);
    k2 = quad_dynamics(x + 0.5*dt*k1, u_input, params);
    k3 = quad_dynamics(x + 0.5*dt*k2, u_input, params);
    k4 = quad_dynamics(x + dt*k3, u_input, params);
    x = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

    X(:,k) = x;
end

final_err = norm(Ref(1:3,end) - X(1:3,end));
fprintf('Final position error (norm): %.6f m\n', final_err);

% Plots
figure('Name','Trajectory (3D)','Color','w'); hold on; grid on; axis equal;
plot3(Ref(1,:),Ref(2,:),Ref(3,:), '--', 'LineWidth',1.6);
plot3(X(1,:),X(2,:),X(3,:), '-', 'LineWidth',2);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
legend('Reference','Actual','Location','best'); title(sprintf('maneuver_freeze (err %.6f m)', final_err));

figure('Name','Position tracking errors','Color','w');
subplot(3,1,1); plot(time, Ref(1,:)-X(1,:),'LineWidth',1.2); ylabel('ex [m]'); grid on;
subplot(3,1,2); plot(time, Ref(2,:)-X(2,:),'LineWidth',1.2); ylabel('ey [m]'); grid on;
subplot(3,1,3); plot(time, Ref(3,:)-X(3,:),'LineWidth',1.2); ylabel('ez [m]'); grid on; xlabel('Time [s]');

figure('Name','Rotor thrusts','Color','w'); hold on; grid on;
cols = lines(4); for i=1:4, plot(time, Urot(i,:),'Color',cols(i,:),'LineWidth',1.1); end
legend('f1','f2','f3','f4','Location','best'); xlabel('Time [s]'); ylabel('Thrust [N]');

figure('Name','Integrator & saturation & maneuver','Color','w');
subplot(3,1,1); plot(time, int_log'); ylabel('pos integrator (filtered)'); legend('ix','iy','iz'); grid on;
subplot(3,1,2); plot(time, double(sat_flag_log)); ylabel('actuator saturation flag'); ylim([-0.1 1.1]); grid on;
subplot(3,1,3); plot(time, double(maneuver_log)); ylabel('maneuver flag'); ylim([-0.1 1.1]); xlabel('Time [s]'); grid on;

animate_quadcopter(X, params, time, Ref);

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function a = wrapAngle(a)
    a = atan2(sin(a), cos(a));
end
