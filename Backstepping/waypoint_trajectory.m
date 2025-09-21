function [p, v, a, yaw] = waypoint_trajectory(t)

waypoints = [0 0 0;
             2 0 4;
             1 2 -2.2;
             -1 0 1.0;
             0 0 2.5;
             0 -0.2 2.0];
    
    t_wp = [0, 5, 10, 15, 20, 25];

    N  = size(waypoints,1);
    xw = waypoints(:,1)'; yw = waypoints(:,2)'; zw = waypoints(:,3)';

    last_dir = waypoints(end,:) - waypoints(end-1,:);
    last_yaw = atan2(last_dir(2), last_dir(1));
    first_dir = waypoints(2,:) - waypoints(1,:);
    first_yaw = atan2(first_dir(2), first_dir(1));

    if t <= t_wp(1)
        p = waypoints(1,:)';
        v = zeros(3,1); a = zeros(3,1);
        yaw = first_yaw;
        return;
    elseif t >= t_wp(end)
        p = waypoints(end,:)';
        v = zeros(3,1); a = zeros(3,1);
        yaw = last_yaw;       
        return;
    end

    ppx   = spline(t_wp, [0, xw, 0]);   
    ppy   = spline(t_wp, [0, yw, 0]);
    ppz   = spline(t_wp, [0, zw, 0]);   

    ppx_d  = ppder(ppx);   ppy_d  = ppder(ppy);   ppz_d  = ppder(ppz);
    ppx_dd = ppder(ppx_d); ppy_dd = ppder(ppy_d); ppz_dd = ppder(ppz_d);

    px = ppval(ppx,   t);  vx = ppval(ppx_d,  t);  ax = ppval(ppx_dd, t);
    py = ppval(ppy,   t);  vy = ppval(ppy_d,  t);  ay = ppval(ppy_dd, t);
    pz = ppval(ppz,   t);  vz = ppval(ppz_d,  t);  az = ppval(ppz_dd, t);

    p = [px; py; pz];
    v = [vx; vy; vz];
    a = [ax; ay; az];

    if norm(v) > 1e-3
        yaw = atan2(vy, vx);
    else
        idx = find(t_wp <= t, 1, 'last');
        if idx < N
            seg_dir = waypoints(idx+1,:) - waypoints(idx,:);
            yaw = atan2(seg_dir(2), seg_dir(1));
        else
            yaw = last_yaw;
        end
    end
end

function ppd = ppder(pp)
    breaks = pp.breaks; coefs = pp.coefs; pieces = pp.pieces; order = pp.order; dim = pp.dim;
    if order <= 1, ppd = mkpp(breaks, zeros(pieces, 1*dim)); return; end
    new_order = order - 1; new_coefs = zeros(pieces, new_order * dim);
    for piece = 1:pieces
        for d = 1:dim
            idxFrom = (d-1)*order + 1; idxTo = d*order; orig = coefs(piece, idxFrom:idxTo);
            deriv = zeros(1, new_order);
            for k = 1:new_order
                power = order - k;
                deriv(k) = orig(k) * power;
            end
            new_coefs(piece, (d-1)*new_order + 1 : d*new_order) = deriv;
        end
    end
    ppd = mkpp(breaks, new_coefs, dim);
end
