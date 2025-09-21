function [a_cmd, info] = controller_mpc_outer(state, p_ref, v_ref, a_ref, yaw_ref, params, ctrl_state, mpc, Phi, Gamma)

p = state(1:3);
v = state(4:6);

dt = mpc.dt;
A = [eye(3), dt*eye(3); zeros(3), eye(3)];
B = [0.5*dt^2*eye(3); dt*eye(3)];
nx = size(A,1); nu = size(B,2);
N = mpc.N;

p_ref_stack = zeros(nx*N,1);
for i=1:N
    p_ref_stack((i-1)*nx + (1:3)) = p_ref;
    p_ref_stack((i-1)*nx + (4:6)) = v_ref;
end

if isempty(Phi) || isempty(Gamma)
    [Phi, Gamma] = build_pred_matrices(N, dt);
end

x0 = [p; v];

Qx = blkdiag(mpc.Qpos, mpc.Qvel);
Qbar = kron(eye(N), Qx);
Rbar = kron(eye(N), mpc.Ru);

H = Gamma' * Qbar * Gamma + Rbar;
H = (H + H')/2;

f = Gamma' * Qbar * (Phi*x0 - p_ref_stack);

lb = repmat([-mpc.a_lat_max; -mpc.a_lat_max; mpc.acc_bounds(1)], N, 1);
ub = repmat([ mpc.a_lat_max;  mpc.a_lat_max; mpc.acc_bounds(2)], N, 1);

opts = optimoptions('quadprog','Display','off','TolFun',1e-6,'TolX',1e-6);
z0 = zeros(3*N,1);
[z_opt, ~, exitflag] = quadprog(H, f, [], [], [], [], lb, ub, z0, opts);

if exitflag ~= 1
    a_cmd = a_ref + params.Kp_pos*(p_ref - p) + params.Kd_pos*(v_ref - v);
    info.exitflag = exitflag;
    info.note = 'quadprog failed, PD fallback';
    return;
end

a_seq = reshape(z_opt,3,N);
a_cmd = a_seq(:,1) + a_ref; 
info.exitflag = exitflag;
info.optval = 0.5*z_opt'*H*z_opt + f'*z_opt;
end
