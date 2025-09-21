clear; close all; clc;

params = quad_params();     
Tsim = 25; dt = 0.005;
time = 0:dt:Tsim;
traj = @(t) helix_trajectory(t);   
% traj = @(t) waypoint_trajectory(t);

[p0, ~, ~, yaw0] = traj(0);
x = zeros(12,1);
x(1:3) = p0;
x(4:6) = [0;0;0];
x(7:9) = [0;0;yaw0];
x(10:12) = [0;0;0];

ctrl_state.pos_int = zeros(3,1); 
ctrl_state.att_int = zeros(3,1);
ctrl_state.phi_des_prev = 0;
ctrl_state.theta_des_prev = 0;
ctrl_state.yaw_des_prev = yaw0;

outer_int.pos = zeros(3,1);     
outer_int.leak_tau = 4.0;         
outer_int.Ki = [0.35; 0.35; 0.6];  

mpc.N = 20;               
mpc.dt = dt;
mpc.Qpos = diag([200,200,400]);   
mpc.Qvel = diag([1,1,1]);
mpc.Ru = 0.01*eye(3);         
mpc.acc_bounds = [-6, 6];
mpc.a_lat_max = params.a_lat_max;

Qx = blkdiag(mpc.Qpos, mpc.Qvel);
mpc.Qf = 5 * Qx;

[Phi, Gamma] = build_pred_matrices(mpc.N, mpc.dt);

Nsim = length(time);
X = zeros(12,Nsim); Urot = zeros(4,Nsim); Ref = zeros(6,Nsim); cost = zeros(1,Nsim);

for k=1:Nsim
    t = time(k);
    [p_ref, v_ref, a_ref, yaw_ref] = traj(t);
    Ref(:,k) = [p_ref; v_ref];

    [a_cmd, info_mpc] = controller_mpc_outer(x, p_ref, v_ref, a_ref, yaw_ref, params, ctrl_state, mpc, Phi, Gamma);

    pos_err = p_ref - x(1:3);
    leak = exp(-dt / max(1e-6, outer_int.leak_tau));
    outer_int.pos = leak * outer_int.pos + pos_err * dt;
    a_int_corr = outer_int.Ki .* outer_int.pos;
    a_int_corr = min(max(a_int_corr, -1.5), 1.5); 
    a_cmd = a_cmd + a_int_corr;

    a_lat = a_cmd(1:2);
    n = norm(a_lat);
    if n > mpc.a_lat_max
        a_cmd(1:2) = a_lat * (mpc.a_lat_max / n);
    end
    a_cmd(3) = clamp(a_cmd(3), mpc.acc_bounds(1), mpc.acc_bounds(2));

    [f_rotors, ctrl_state, ctrl_info] = controller_inner_from_acc(x, a_cmd, yaw_ref, ctrl_state, params, dt);

    Urot(:,k) = f_rotors;
    cost(k) = norm(p_ref - x(1:3));

    u_input.f = f_rotors;
    u_input.yaw_ref = yaw_ref;
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
legend('Reference','Actual'); title('3D trajectory tracking');

figure('Name','Position tracking errors');
subplot(3,1,1); plot(time, Ref(1,:)-X(1,:)); ylabel('ex [m]'); grid on;
subplot(3,1,2); plot(time, Ref(2,:)-X(2,:)); ylabel('ey [m]'); grid on;
subplot(3,1,3); plot(time, Ref(3,:)-X(3,:)); ylabel('ez [m]'); grid on;
xlabel('Time [s]');

figure('Name','Rotor thrusts'); plot(time, Urot'); legend('f1','f2','f3','f4'); ylabel('Thrust [N]'); xlabel('Time [s]'); grid on;

animate_quadcopter(X, params, time, Ref);

fprintf('Final position error (norm): %.4f m\n', norm(Ref(1:3,end)-X(1:3,end)));

function y = clamp(x, lo, hi), y = min(max(x,lo),hi); end
function y = clamp_vec(x, lo, hi), y = min(max(x,lo),hi); end
function a = wrapAngle(a), a = atan2(sin(a), cos(a)); end
function out = slew(prev, target, rate_max, dt)
    delta = wrapAngle(target - prev);
    step = clamp(delta, -rate_max*dt, rate_max*dt);
    out = prev + step;
    out = wrapAngle(out);
end
function a = wrapToPi(a), a = atan2(sin(a), cos(a)); end
