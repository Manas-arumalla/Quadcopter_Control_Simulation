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

    params.Kp_pos = diag([5.9157, 6.9682, 8.0949]);
    params.Kd_pos = diag([5.7135, 2.5004, 2.5957]);
    params.Ki_pos = diag([0.6265, 0.0166, 0.1872]);

    params.Kp_att = diag([103.6124, 149.7349, 50.1208]);
    params.Kd_att = diag([2.1297, 2.5340, 3.7858]);
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
