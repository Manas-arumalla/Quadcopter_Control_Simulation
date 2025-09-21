function [p, v, a, yaw] = helix_trajectory(t)

    R = 1.2;       
    wz = 0.4;     
    z0 = 1.0;
    z_amp = 0.4;
    wz_z = 0.5;      

    px = R * cos(wz * t);
    py = R * sin(wz * t);
    pz = z0 + z_amp * sin(wz_z * t);
    p = [px; py; pz];

    vx = -R * wz * sin(wz * t);
    vy =  R * wz * cos(wz * t);
    vz = z_amp * wz_z * cos(wz_z * t);
    v = [vx; vy; vz];

    ax = -R * (wz^2) * cos(wz * t);
    ay = -R * (wz^2) * sin(wz * t);
    az = -z_amp * (wz_z^2) * sin(wz_z * t);
    a = [ax; ay; az];

    yaw = atan2(vy, vx);
end
