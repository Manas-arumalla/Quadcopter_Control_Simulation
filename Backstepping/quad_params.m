function params = quad_params()
    params.m = 1.2;
    params.g = 9.81;
    params.l = 0.225;
    params.kf = 2.98e-6 * 1e6;
    params.km = 1.14e-7 * 1e6;
    params.I  = diag([0.014, 0.014, 0.028]);
    params.drag_coeff = 0.10;

    params.f_min = 0;
    params.f_max = 8;            
    params.eps   = 1e-6;

    l  = params.l; km = params.km;
    params.M = [ 1 ,  1 ,  1 ,  1;
                 l , -l , -l ,  l;
                -l , -l ,  l ,  l;
                -km,  km, -km,  km];
    params.M_inv = inv(params.M);

    params.Kp_pos = diag([0, 0, 0.2772 ]);
    params.Kd_pos = diag([0, 0, 0]);
    params.Ki_pos = diag([10, 10, 8]);

    params.Kp_att = diag([0.5, 0.5358, 0.2]);
    params.Kd_att = diag([2.5, 2.5, 1.1]);
    params.Ki_att = diag([0, 0, 0]);

    params.pos_int_leak_tau = 3.0;

    params.tilt_max    = deg2rad(28);    
    params.a_lat_max   = params.g * tan(params.tilt_max)*0.95;  
    params.a_up_max    = 6.0;       
    params.a_dn_max    = 6.0;   

    params.att_rate_max = deg2rad([140; 140; 100]); 

    params.int_lim_pos = [0.6; 0.6; 0.8];
    params.int_lim_att = deg2rad([10; 10; 5]);
end
