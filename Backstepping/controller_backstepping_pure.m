function [f_rotors, ctrl_state, info] = controller_backstepping_pure(state, p_ref, v_ref, a_ref, yaw_ref, ctrl_state, params, dt)

p = state(1:3);
v = state(4:6);
phi = state(7);
theta = state(8);
psi = state(9);
omega = state(10:12); 

Kp_pos = diag([4.5, 4.5, 9.0]);
Kd_pos = diag([3.5, 3.5, 5.0]);

K_R = diag([60, 60, 40]);
K_omega = diag([6, 6, 3]);

if ~isfield(params,'m'), params.m = 1.2; end
if ~isfield(params,'g'), params.g = 9.81; end
if ~isfield(params,'I'), params.I = diag([0.014,0.014,0.028]); end
I = params.I;

if ~isfield(params,'f_min'), params.f_min = 0; end
if ~isfield(params,'f_max'), params.f_max = 8; end
if ~isfield(params,'eps'), params.eps = 1e-6; end

pos_err = p - p_ref;
vel_err = v - v_ref;

a_des = a_ref - Kd_pos * vel_err - Kp_pos * pos_err; 

F_des = params.m * (a_des + [0;0;params.g]);

F_norm = norm(F_des);
if F_norm < 1e-6
    b3_des = [0;0;1];
else
    b3_des = F_des / F_norm; 
end

U1_des = F_norm;              
total_max = 4 * params.f_max;
U1_des = min(max(U1_des, 0), total_max);

yaw = yaw_ref;
c1 = cos(yaw); s1 = sin(yaw);
x_c = [c1; s1; 0]; 

b1_temp = cross(x_c, b3_des);
if norm(b1_temp) < 1e-6

    b1_temp = [1;0;0];
end
b1_des = b1_temp / norm(b1_temp);
b2_des = cross(b3_des, b1_des);

R_des = [b1_des, b2_des, b3_des];

R = eul2rotm_vec([psi, theta, phi]); 

R_err = R_des' * R;
S = 0.5 * (R_err - R_err');
e_R = vee(S);    % 3x1

omega_des = [0;0;0];

e_omega = omega - R' * R_des * omega_des;

tau = - K_R * e_R - K_omega * e_omega + cross(omega, I * omega);

U = [U1_des; tau];

f_unsat = params.M_inv * U;

if any(~isfinite(f_unsat))
    f_unsat = repmat((params.m*params.g)/4, 4, 1);
end

f = min(max(f_unsat, params.f_min), params.f_max);

if all(f==0)
    f = repmat((params.m*params.g)/4, 4, 1);
end

if ~isstruct(ctrl_state)
    ctrl_state = struct();
end

f_rotors = f;
ctrl_state = ctrl_state;
if nargout > 2
    info.U = U;
    info.R_des = R_des;
    info.R = R;
    info.e_R = e_R;
    info.e_omega = e_omega;
    info.F_des = F_des;
    info.U1_des = U1_des;
end
end

%% -------------------- helper functions --------------------
function R = eul2rotm_vec(ztyx)
psi = ztyx(1); theta = ztyx(2); phi = ztyx(3);
Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
R = Rz * Ry * Rx;
end

function v = vee(S)
v = [S(3,2); S(1,3); S(2,1)];
end
