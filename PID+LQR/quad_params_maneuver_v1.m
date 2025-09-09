function params = quad_params_maneuver_v1()

params.m = 1.2; params.g = 9.81; params.l = 0.225;
params.kf = 2.98e-6 * 1e6; params.km = 1.14e-7 * 1e6;
params.I  = diag([0.014, 0.014, 0.028]);
params.drag_coeff = 0.10;

params.f_min = 0; params.f_max = 8; params.eps = 1e-6;

l  = params.l; km = params.km;
params.M = [ 1 ,  1 ,  1 ,  1;
             l , -l , -l ,  l;
            -l , -l ,  l ,  l;
            -km,  km, -km,  km];
params.M_inv = inv(params.M);

params.Kp_pos = diag([4.8, 4.8, 10.8]);
params.Kd_pos = diag([3.8, 3.8, 5.0]);
params.Ki_pos = diag([0.015, 0.015, 0.05]);

params.pos_int_leak_tau = 200.0;
params.int_lim_pos = [0.7; 0.7; 0.9];

params.int_lim_att = deg2rad([10;10;5]);
params.tilt_max    = deg2rad(28);
params.a_up_max = 6.0; params.a_dn_max = 6.0;
params.a_lat_max = params.g * tan(params.tilt_max) * 0.95;
params.att_rate_max = deg2rad([220;220;140]);
end