function [f_rotors, ctrl, saturated, sat_heavy_flag] = controller_maneuver_adaptive(...
    x_state, p_ref, v_ref, a_ref, yaw_ref, ctrl, params, ...
    dt, K_att, Ki_att, att_int_lim, tau_max, ...
    kb_aw_nom, pos_int_freeze_thresh, pos_int_rate_limit, sat_bleed, ...
    U1_rate_max, f_rate_max, att_rate_limits, tau_rot, D_omega, ...
    Klead_pos, Ki_scale, int_filter_tau, PD_scale, integrator_freeze)

pos = x_state(1:3); vel = x_state(4:6);
phi = x_state(7); theta = x_state(8); psi = x_state(9);
omega = x_state(10:12);

pos_err = p_ref - pos;
vel_err = v_ref - vel;

% Leaky integrator base
leak_nom = exp(-dt / max(1e-6, params.pos_int_leak_tau));

% Integrator update
if integrator_freeze
    pos_int_tmp = leak_nom * ctrl.pos_int;
else
    pos_int_tmp = leak_nom * ctrl.pos_int + pos_err * dt;
end

% Lead (anticipatory)
lead_term = -Klead_pos * vel_err * dt;
pos_int_tmp = pos_int_tmp + lead_term;

% Internal integrator soft-clamp
for i=1:3
    pos_int_tmp(i) = sat_smooth(pos_int_tmp(i), params.int_lim_pos(i), sat_bleed);
end

% Outer PID
Kp_eff = PD_scale * params.Kp_pos;
Kd_eff = PD_scale * params.Kd_pos;
a_cmd = a_ref + Kp_eff * pos_err + Kd_eff * vel_err + (Ki_scale * params.Ki_pos) * pos_int_tmp;

% Accel clamps & lateral limit
a_cmd(3) = clamp(a_cmd(3), -params.a_dn_max, params.a_up_max);
a_lat = a_cmd(1:2);
if norm(a_lat) > params.a_lat_max
    a_cmd(1:2) = a_lat * (params.a_lat_max / norm(a_lat));
end

% Desired force & tilt computation
F_des = params.m * (a_cmd + [0;0;params.g]);

% yaw-rate limit & desired yaw integration 
desired_yaw = wrapAngle(yaw_ref);
yaw_prev = ctrl.yaw_des_prev;
max_yaw_rate = att_rate_limits(3);
delta_yaw = wrapAngle(desired_yaw - yaw_prev);
delta_yaw = clamp(delta_yaw, -max_yaw_rate*dt, max_yaw_rate*dt);
psi_des = wrapAngle(yaw_prev + delta_yaw);

% desired roll/pitch
phi_des_raw   = atan2( F_des(1)*sin(psi_des) - F_des(2)*cos(psi_des), F_des(3) );
theta_des_raw = atan2( F_des(1)*cos(psi_des) + F_des(2)*sin(psi_des), F_des(3) );
phi_des_raw = clamp(phi_des_raw, -params.tilt_max, params.tilt_max);
theta_des_raw = clamp(theta_des_raw, -params.tilt_max, params.tilt_max);

% Slew & LPF attitude commands
rate = att_rate_limits;
phi_des = slew(ctrl.phi_des_prev, phi_des_raw, rate(1), dt);
theta_des = slew(ctrl.theta_des_prev, theta_des_raw, rate(2), dt);
ctrl.phi_des_prev = phi_des; ctrl.theta_des_prev = theta_des; ctrl.yaw_des_prev = psi_des;

tau_f = ctrl.tau_ang_cmd_filter;
if tau_f > 1e-6
    alpha = dt / (tau_f + dt);
    ctrl.phi_des_f = (1-alpha)*ctrl.phi_des_f + alpha*phi_des;
    ctrl.theta_des_f = (1-alpha)*ctrl.theta_des_f + alpha*theta_des;
    phi_cmd = ctrl.phi_des_f; theta_cmd = ctrl.theta_des_f;
else
    phi_cmd = phi_des; theta_cmd = theta_des;
end

x_att = [phi; theta; psi; omega(1); omega(2); omega(3)];
x_att_ref = [phi_cmd; theta_cmd; psi_des; 0; 0; 0];

angle_err = [phi_cmd - phi; theta_cmd - theta; wrapAngle(psi_des - psi)];
att_int_tmp = ctrl.att_int + angle_err * dt;
att_int_tmp = min(max(att_int_tmp, -att_int_lim), att_int_lim);

tau_lqr = - K_att * (x_att - x_att_ref);
tau_int = Ki_att * att_int_tmp;
tau_visc = - D_omega * omega;
tau_cmd = tau_lqr + tau_int + tau_visc;

% attitude anti-windup & store
tau_cmd_sat = min(max(tau_cmd, -tau_max), tau_max);
sat_att = any((tau_cmd < -tau_max) | (tau_cmd > tau_max));
if ~sat_att
    ctrl.att_int = att_int_tmp;
else
    ctrl.att_int = 0.995 * ctrl.att_int + 0.005 * att_int_tmp;
end

% Collective feedforward and U1 slew
U1_ff = params.m * (params.g + a_cmd(3));
c_des = max(0.25, cos(phi_cmd) * cos(theta_cmd));
U1_des_nom = clamp(U1_ff / c_des, 0, 4*params.f_max);

U1_prev = ctrl.U1_prev;
max_dU1 = U1_rate_max * dt;
U1_des = clamp(U1_des_nom, U1_prev - max_dU1, U1_prev + max_dU1);
U1_des = clamp(U1_des, 0, 4*params.f_max);
ctrl.U1_prev = U1_des;

U = [U1_des; tau_cmd_sat(:)];
f_unsat = params.M_inv * U;
f_sat = min(max(f_unsat, params.f_min), params.f_max);

% Anti-windup back-calc
U1_unsat = sum(f_unsat);
U1_sat = sum(f_sat);
R_des = eul2rotm([psi_des, theta_cmd, phi_cmd]);
F_world_des = R_des * [0;0;U1_unsat];
F_world_act = R_des * [0;0;U1_sat];
F_short = F_world_des - F_world_act;
a_short = F_short / params.m;

% Adaptive back-calc + leak
sat_heavy_flag = any((f_unsat < params.f_min - params.eps) | (f_unsat > params.f_max + params.eps));
if sat_heavy_flag
    kb_aw = kb_aw_nom * 0.2;
    leak = exp(-dt / (max(1e-6, params.pos_int_leak_tau) * 0.2));
else
    kb_aw = kb_aw_nom;
    leak = exp(-dt / max(1e-6, params.pos_int_leak_tau));
end

pos_int_backcalc = pos_int_tmp + kb_aw * a_short * dt;
for i=1:3
    pos_int_backcalc(i) = sat_smooth(pos_int_backcalc(i), params.int_lim_pos(i), sat_bleed);
end

% Conservative integrator update when saturated/large error/frozen
if sat_heavy_flag || (norm(pos_err) > pos_int_freeze_thresh) || integrator_freeze
    desired_pos_int = 0.995 * ctrl.pos_int + 0.005 * pos_int_backcalc;
else
    desired_pos_int = pos_int_backcalc;
end

% integrator rate limiting
max_int_delta = pos_int_rate_limit * dt;
delta = desired_pos_int - ctrl.pos_int;
for i=1:3
    delta(i) = clamp(delta(i), -max_int_delta(i), max_int_delta(i));
end
ctrl.pos_int = ctrl.pos_int + delta;

% integrator internal LPF
alpha_if = dt / (int_filter_tau + dt);
ctrl.pos_int_f = (1-alpha_if)*ctrl.pos_int_f + alpha_if * ctrl.pos_int;
for i=1:3
    ctrl.pos_int_f(i) = sat_smooth(ctrl.pos_int_f(i), params.int_lim_pos(i), sat_bleed);
end

% additional bleed when frozen
if integrator_freeze
    ctrl.pos_int_f = 0.995 * ctrl.pos_int_f;
end

% copy filtered integrator back
ctrl.pos_int = ctrl.pos_int_f;

% rotor slew & LPF
max_df = f_rate_max * dt;
f_prev = ctrl.f_prev;
f_limited = zeros(4,1);
for i=1:4
    f_limited(i) = clamp(f_sat(i), f_prev(i) - max_df, f_prev(i) + max_df);
end

alpha_r = dt / (tau_rot + dt);
f_filtered = (1-alpha_r)*f_prev + alpha_r * f_limited;
ctrl.f_prev = f_filtered;

f_rotors = f_filtered;
saturated = sat_att | sat_heavy_flag;
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
end

function s = sat_smooth(x, limit, bleed)
    if abs(x) <= limit
        s = x;
    else
        s = sign(x) * (limit + bleed * (abs(x) - limit));
    end
end

function y = slew(prev, target, rate_max, dt)
    delta = wrapAngle(target - prev);
    step = clamp(delta, -rate_max*dt, rate_max*dt);
    y = prev + step; y = wrapAngle(y);
end

function a = wrapAngle(a)
    a = atan2(sin(a), cos(a));
end

function R = eul2rotm(ztyx)
    psi = ztyx(1); theta = ztyx(2); phi = ztyx(3);
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R = Rz * Ry * Rx;
end
