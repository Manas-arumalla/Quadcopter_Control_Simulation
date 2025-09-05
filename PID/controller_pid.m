function [f_rotors, ctrl_state, info] = controller_pid(state, p_ref, v_ref, a_ref, yaw_ref, ctrl_state, params, dt)

    pos = state(1:3); vel = state(4:6);
    phi = state(7);   theta = state(8);  psi = state(9);
    omega = state(10:12);

    % Outer loop (position)
    pos_err = p_ref - pos;
    vel_err = v_ref - vel;

    % tentative integral (clamped)
    leak = exp(-dt / max(1e-6, params.pos_int_leak_tau));
    pos_int_tmp = leak * ctrl_state.pos_int + pos_err * dt;
    pos_int_tmp = clamp_vec(pos_int_tmp, -params.int_lim_pos, params.int_lim_pos);

    a_cmd = a_ref + params.Kp_pos*pos_err + params.Kd_pos*vel_err + params.Ki_pos*pos_int_tmp;

    a_cmd(3) = clamp(a_cmd(3), -params.a_dn_max, params.a_up_max);

    a_lat = a_cmd(1:2);
    n = norm(a_lat);
    if n > params.a_lat_max
        a_cmd(1:2) = a_lat * (params.a_lat_max / n);
    end

    F_des = params.m * (a_cmd + [0;0;params.g]);

    c = cos(phi)*cos(theta);
    c = max(0.3, c);
    U1_des = F_des(3) / c;
    U1_des = max(U1_des, 0);
    U1_des = min(U1_des, 4*params.f_max);

    phi_des_raw   = atan2( F_des(1)*sin(yaw_ref) - F_des(2)*cos(yaw_ref), F_des(3) );
    theta_des_raw = atan2( F_des(1)*cos(yaw_ref) + F_des(2)*sin(yaw_ref), F_des(3) );

    phi_des_raw   = clamp(phi_des_raw,   -params.tilt_max, params.tilt_max);
    theta_des_raw = clamp(theta_des_raw, -params.tilt_max, params.tilt_max);

    rate = params.att_rate_max;
    phi_des   = slew(ctrl_state.phi_des_prev,   phi_des_raw,   rate(1), dt);
    theta_des = slew(ctrl_state.theta_des_prev, theta_des_raw, rate(2), dt);
    psi_des   = slew(ctrl_state.yaw_des_prev,   wrapToPi(yaw_ref), rate(3), dt);
    ctrl_state.phi_des_prev   = phi_des;
    ctrl_state.theta_des_prev = theta_des;
    ctrl_state.yaw_des_prev   = psi_des;

    % Inner loop (attitude)
    angle_err = [phi_des - phi; theta_des - theta; wrapToPi(psi_des - psi)];

    att_int_tmp = ctrl_state.att_int + angle_err*dt;
    att_int_tmp = clamp_vec(att_int_tmp, -params.int_lim_att, params.int_lim_att);

    tau = params.Kp_att*angle_err - params.Kd_att*omega + params.Ki_att*att_int_tmp;

    U = [U1_des; tau];
    f_unsat = params.M_inv * U;

    f = min(max(f_unsat, params.f_min), params.f_max);

    sat_any = any((f_unsat < params.f_min-params.eps) | (f_unsat > params.f_max+params.eps));
    if ~sat_any
        ctrl_state.pos_int = pos_int_tmp;
        ctrl_state.att_int = att_int_tmp;
    end

    f_rotors = f;
    if nargout > 2
        info.U = U; info.angle_err = angle_err; info.a_cmd = a_cmd;
    end
end

function y = clamp(x, lo, hi), y = min(max(x,lo),hi); end
function y = clamp_vec(x, lo, hi), y = min(max(x,lo),hi); end
function y = slew(prev, target, rate_max, dt)
    delta = wrapAngle(target - prev);
    step  = clamp(delta, -rate_max*dt, rate_max*dt);
    y = prev + step;
    y = wrapAngle(y);
end
function a = wrapAngle(a), a = atan2(sin(a), cos(a)); end
