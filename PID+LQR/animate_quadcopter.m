function animate_quadcopter(X, params, time, Ref)

figure('Name','Quadcopter animation'); axis equal; grid on; hold on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
view(45,20);
plot3(Ref(1,:), Ref(2,:), Ref(3,:), '--', 'LineWidth',1); 

max_range = max(max(abs(X(1:3,:)))) + 1;
xlim([-max_range, max_range]); ylim([-max_range, max_range]);
zlim([-0.2, max_range+1]);

N = size(X,2);
h_body = plot3(0,0,0,'-o','MarkerSize',6,'MarkerFaceColor','k');
h_arm1 = plot3([0 0],[0 0],[0 0],'LineWidth',2);
h_arm2 = plot3([0 0],[0 0],[0 0],'LineWidth',2);

step = max(1, floor(N/400)); 
for k=1:step:N
    pos = X(1:3,k);
    phi = X(7,k); theta = X(8,k); psi = X(9,k);
    R = eul2rotm([psi, theta, phi]);
    arm = params.l;
    r1 = R * [ arm; 0; 0] + pos;
    r2 = R * [ 0; -arm; 0] + pos;
    r3 = R * [-arm; 0; 0] + pos;
    r4 = R * [ 0;  arm; 0] + pos;

    set(h_body,'XData',pos(1),'YData',pos(2),'ZData',pos(3));
    set(h_arm1,'XData',[r1(1) r3(1)], 'YData',[r1(2) r3(2)], 'ZData',[r1(3) r3(3)]);
    set(h_arm2,'XData',[r2(1) r4(1)], 'YData',[r2(2) r4(2)], 'ZData',[r2(3) r4(3)]);
    drawnow;
end
end
