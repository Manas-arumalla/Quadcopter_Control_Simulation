function J = sim_cost_backstepping_improved(x, controller_name)

params = quad_params();
params.Kp_pos = diag(x(1:3));
params.Kd_pos = diag(x(4:6));
params.K_R = diag(x(7:9));
params.K_omega = diag(x(10:12));

Tsim = 12;  
dt = 0.005;
time = 0:dt:Tsim;
N = numel(time);

traj = @(t) helix_trajectory(t);
[p0, v0, a0, yaw0] = traj(0);

xstate = zeros(12,1);
xstate(1:3) = p0;         
xstate(4:6) = v0;         
xstate(7:9) = [0;0;yaw0];  
xstate(10:12) = [0;0;0];

ctrl_state.pos_int = zeros(3,1);
ctrl_state.att_int = zeros(3,1);
ctrl_state.phi_des_prev = 0;
ctrl_state.theta_des_prev = 0;
ctrl_state.yaw_des_prev = yaw0;

J_pos = 0; J_att = 0; J_ctrl = 0;
max_allowed_pos = 200;

for k = 1:N
    t = time(k);
    [p_ref, v_ref, a_ref, yaw_ref] = traj(t);

    try
        [f_rotors, ctrl_state, info] = feval(controller_name, xstate, p_ref, v_ref, a_ref, yaw_ref, ctrl_state, params, dt);
    catch
        J = 1e9; return;
    end

    if any(~isfinite(f_rotors)) || any(isnan(f_rotors))
        J = 1e9; return;
    end

    u_input.f = f_rotors;
    k1 = quad_dynamics(xstate, u_input, params);
    k2 = quad_dynamics(xstate + 0.5*dt*k1, u_input, params);
    k3 = quad_dynamics(xstate + 0.5*dt*k2, u_input, params);
    k4 = quad_dynamics(xstate + dt*k3, u_input, params);
    xstate = xstate + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);

    pos_err = p_ref - xstate(1:3);
    J_pos = J_pos + (pos_err' * pos_err) * dt;

    if exist('info','var') && isfield(info,'e_R')
        eR = info.e_R;
    else

        if exist('info','var') && isfield(info,'angle_err')
            eR = info.angle_err;
        else
            eR = [0;0;0];
        end
    end
    J_att = J_att + (eR' * eR) * dt;

    J_ctrl = J_ctrl + (f_rotors' * f_rotors) * dt;

    if any(abs(xstate(1:3)) > max_allowed_pos) || ~isfinite(xstate(1))
        J = 1e9; return;
    end
end

w_pos = 1.0;      
w_att = 0.5;     
w_ctrl = 1e-3;   

norm_fac = max(1e-6, Tsim);
J = w_pos * (J_pos / norm_fac) + w_att * (J_att / norm_fac) + w_ctrl * (J_ctrl / norm_fac);

if ~isfinite(J) || J < 0
    J = 1e9;
end
end
