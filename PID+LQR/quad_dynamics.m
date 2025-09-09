function xdot = quad_dynamics(state, u_input, params)

pos = state(1:3); vel = state(4:6);
euler = state(7:9); omega = state(10:12);
phi = euler(1); theta = euler(2); psi = euler(3);

R = eul2rotm([psi, theta, phi]);

f = u_input.f;
U = params.M * f;        
U1 = U(1); tau = U(2:4);

F_body = [0; 0; U1];
F_world = R * F_body;

F_drag = -params.drag_coeff * vel;
accel = (1/params.m) * (F_world + F_drag) + [0;0; -params.g];

I = params.I;
omega_dot = I \ (tau - cross(omega, I*omega));

T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
     0, cos(phi), -sin(phi);
     0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
euler_dot = T * omega;

xdot = zeros(12,1);
xdot(1:3) = vel;
xdot(4:6) = accel;
xdot(7:9) = euler_dot;
xdot(10:12) = omega_dot;
end

function R = eul2rotm(ztyx)
    psi = ztyx(1); theta = ztyx(2); phi = ztyx(3);
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R = Rz * Ry * Rx;
end
