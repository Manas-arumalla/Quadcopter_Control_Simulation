function J = sim_cost_for_gains(x, controller_type)

Tsim = 12; 
dt = 0.01;
time = 0:dt:Tsim;
N = length(time);

traj = @(t) helix_trajectory(t); 


Kp_pos = diag([x(1), x(2), x(3)]);
Kd_pos = diag([x(4), x(5), x(6)]);
Ki_pos = diag([x(7), x(8), x(9)]);
Kp_att = diag([x(10), x(11), x(12)]);
Kd_att = diag([x(13), x(14), x(15)]);

params.Kp_pos = Kp_pos;
params.Kd_pos = Kd_pos;
params.Ki_pos = Ki_pos;
params.Kp_att = Kp_att;
params.Kd_att = Kd_att;
params.Ki_att = diag([0,0,0]); 
controller_handle = @controller_pid;


[p0, v0, a0, yaw0] = traj(0);
x_state = zeros(12,1);
x_state(1:3) = p0;
x_state(4:6) = v0;
x_state(7:9) = [0;0;yaw0];
x_state(10:12) = [0;0;0];

ctrl_state.pos_int = zeros(3,1);
ctrl_state.att_int = zeros(3,1);
ctrl_state.phi_des_prev = 0;
ctrl_state.theta_des_prev = 0;
ctrl_state.yaw_des_prev = yaw0;

J = 0;
max_allowed_pos = 100;

for k = 1:N
    t = time(k);
    [p_ref, v_ref, a_ref, yaw_ref] = traj(t);
    try
        [f_rotors, ctrl_state, ctrl_info] = controller_handle(x_state, p_ref, v_ref, a_ref, yaw_ref, ctrl_state, params, dt);
    catch ME
        J = 1e8;
        return;
    end
    if any(~isfinite(f_rotors)) || any(isnan(f_rotors))
        J = 1e8; return;
    end

    u_input.f = f_rotors;
    k1 = quad_dynamics(x_state, u_input, params);
    x_state = x_state + dt * k1; 

    pos_err = (p_ref - x_state(1:3));
    J = J + (pos_err' * pos_err) * dt;

    if any(abs(x_state(1:3)) > max_allowed_pos) || ~isfinite(x_state(1))
        J = 1e8; return;
    end
end
end
