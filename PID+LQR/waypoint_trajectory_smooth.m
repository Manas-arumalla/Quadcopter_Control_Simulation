function [p, v, a, yaw] = waypoint_trajectory_smooth(t)

waypoints = [0,0,0; 1,0,1.2; 1,2,2.2; -1,4,1.0; 0,0,1.5; 0,0.2,2.0];
t_wp = [0,5,10,15,20,25];

if t <= t_wp(1)
    p = waypoints(1,:)'; v=zeros(3,1); a=zeros(3,1);
    yaw = atan2(waypoints(2,2)-waypoints(1,2), waypoints(2,1)-waypoints(1,1));
    return;
elseif t >= t_wp(end)
    p = waypoints(end,:)'; v=zeros(3,1); a=zeros(3,1);
    yaw = atan2(waypoints(end,2)-waypoints(end-1,2), waypoints(end,1)-waypoints(end-1,1));
    return;
end

xw = waypoints(:,1)'; yw = waypoints(:,2)'; zw = waypoints(:,3)';
ppx = spline(t_wp, [0, xw, 0]); ppy = spline(t_wp, [0, yw, 0]); ppz = spline(t_wp, [0, zw, 0]);

ppx_d = ppder(ppx); ppy_d = ppder(ppy); ppz_d = ppder(ppz);
ppx_dd = ppder(ppx_d); ppy_dd = ppder(ppy_d); ppz_dd = ppder(ppz_d);

h = 5e-4;
vx = ppval(ppx_d, t); vy = ppval(ppy_d, t); vz = ppval(ppz_d, t);
ax = ppval(ppx_dd, t); ay = ppval(ppy_dd, t); az = ppval(ppz_dd, t);

px = ppval(ppx, t); py = ppval(ppy, t); pz = ppval(ppz, t);

p = [px; py; pz];
v = [vx; vy; vz];
a = [ax; ay; az];

if norm(v) > 1e-3
    yaw = atan2(v(2), v(1));
else
    idx = find(t_wp <= t, 1, 'last');
    if idx < size(waypoints,1)
        seg_dir = waypoints(idx+1,:) - waypoints(idx,:);
        yaw = atan2(seg_dir(2), seg_dir(1));
    else
        yaw = atan2(waypoints(end,2)-waypoints(end-1,2), waypoints(end,1)-waypoints(end-1,1));
    end
end
end


