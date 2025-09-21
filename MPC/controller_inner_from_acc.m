function [f_rotors, ctrl_state, info] = controller_inner_from_acc(state, a_cmd, yaw_ref, ctrl_state, params, dt)

pos = state(1:3); vel = state(4:6);
phi = state(7); theta = state(8); psi = state(9);
omega = state(10:12);

F_des = params.m * (a_cmd + [0;0;params.g]);

c = cos(phi)*cos(theta); c = max(0.3, c);
U1_des = F_des(3) / c;
U1_des = max(U1_des, 0);
U1_des = min(U1_des, 4*params.f_max);

phi_des_raw = atan2( F_des(1)*sin(yaw_ref) - F_des(2)*cos(yaw_ref), F_des(3) );
theta_des_raw = atan2( F_des(1)*cos(yaw_ref) + F_des(2)*sin(yaw_ref), F_des(3) );

phi_des = clamp(phi_des_raw, -params.tilt_max, params.tilt_max);
theta_des = clamp(theta_des_raw, -params.tilt_max, params.tilt_max);
psi_des = wrapToPi(yaw_ref);

rate = params.att_rate_max;
phi_des = slew(ctrl_state.phi_des_prev, phi_des, rate(1), dt);
theta_des = slew(ctrl_state.theta_des_prev, theta_des, rate(2), dt);
psi_des = slew(ctrl_state.yaw_des_prev, psi_des, rate(3), dt);
ctrl_state.phi_des_prev = phi_des;
ctrl_state.theta_des_prev = theta_des;
ctrl_state.yaw_des_prev = psi_des;

angle_err = [phi_des - phi; theta_des - theta; wrapToPi(psi_des - psi)];
att_int_tmp = ctrl_state.att_int + angle_err * dt;
att_int_tmp = clamp_vec(att_int_tmp, -params.int_lim_att, params.int_lim_att);
tau = params.Kp_att*angle_err - params.Kd_att*omega + params.Ki_att*att_int_tmp;

U = [U1_des; tau];
f_unsat = params.M_inv * U;
f = min(max(f_unsat, params.f_min), params.f_max);

sat_any = any((f_unsat < params.f_min-params.eps) | (f_unsat > params.f_max+params.eps));
if ~sat_any
    ctrl_state.att_int = att_int_tmp;
end

f_rotors = f;
info.U = U;
info.angle_err = angle_err;
info.a_cmd = a_cmd;
end

% ---------- local helpers ----------
function y = clamp(x, lo, hi), y = min(max(x,lo),hi); end
function y = clamp_vec(x, lo, hi), y = min(max(x,lo),hi); end
function out = slew(prev, target, rate_max, dt)
    delta = wrapAngle(target - prev);
    step = clamp(delta, -rate_max*dt, rate_max*dt);
    out = prev + step;
    out = wrapAngle(out);
end
function a = wrapAngle(a), a = atan2(sin(a), cos(a)); end
function a = wrapToPi(a), a = atan2(sin(a), cos(a)); end
