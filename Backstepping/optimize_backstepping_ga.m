clear; close all; clc;

controller_name = 'controller_backstepping_pure'; 
use_parallel = false;  

baseline = [4.5, 4.5, 9.0,   3.5, 3.5, 5.0,   80, 80, 45,   4.0, 4.0, 2.0];
nvars = numel(baseline);

lb = max(1e-3, baseline .* 0.5);
ub = baseline .* 1.8;

popSize = 48;      
nGenerations = 30;     

InitialPopulation = zeros(popSize, nvars);
for i=1:popSize

    pert = baseline .* (1 + 0.10 * randn(size(baseline)));
    pert = max(pert, lb);
    pert = min(pert, ub);
    InitialPopulation(i,:) = pert;
end

options = optimoptions('ga', ...
    'PopulationSize', popSize, ...
    'MaxGenerations', nGenerations, ...
    'InitialPopulationMatrix', InitialPopulation, ...
    'Display', 'iter', ...
    'PlotFcn', {@gaplotbestf}, ...
    'UseParallel', use_parallel, ...
    'FunctionTolerance', 1e-6);

objfun = @(x) sim_cost_backstepping_improved(x, controller_name);

rng(2,'twister');

fprintf('Starting GA: pop=%d, gens=%d\n', popSize, nGenerations);
[xbest, fbest, exitflag, output, population, scores] = ga(objfun, nvars, [],[],[],[], lb, ub, [], options);

fprintf('GA finished. Best cost = %.6f\n', fbest);
disp('Best gains (xbest):');
disp(xbest);

save('ga_backstepping_results.mat', 'xbest', 'fbest', 'population', 'scores', 'output');

fprintf('Running validation simulation with xbest (longer sim)...\n');
validate_with_gains(xbest, controller_name);

%% ---------- local helper: validation simulation ----------
function validate_with_gains(xbest, controller_name)
    params = quad_params();
    params.Kp_pos = diag(xbest(1:3));
    params.Kd_pos = diag(xbest(4:6));
    params.K_R    = diag(xbest(7:9));
    params.K_omega= diag(xbest(10:12));

    Tsim = 25; dt = 0.005; time = 0:dt:Tsim; N = numel(time);
    traj = @(t) helix_trajectory(t);

    [p0, v0, a0, yaw0] = traj(0);
    xstate = zeros(12,1);
    xstate(1:3) = p0; xstate(4:6) = v0; xstate(7:9) = [0;0;yaw0]; xstate(10:12) = 0;

    ctrl_state.pos_int = zeros(3,1);
    ctrl_state.att_int = zeros(3,1);
    ctrl_state.phi_des_prev = 0;
    ctrl_state.theta_des_prev = 0;
    ctrl_state.yaw_des_prev = yaw0;

    X = zeros(12,N); Ref = zeros(6,N); Urot = zeros(4,N);
    for k=1:N
        t = time(k);
        [p_ref, v_ref, a_ref, yaw_ref] = traj(t);
        [f_rotors, ctrl_state, info] = feval(controller_name, xstate, p_ref, v_ref, a_ref, yaw_ref, ctrl_state, params, dt);
        u_input.f = f_rotors;
        k1 = quad_dynamics(xstate, u_input, params);
        k2 = quad_dynamics(xstate + 0.5*dt*k1, u_input, params);
        k3 = quad_dynamics(xstate + 0.5*dt*k2, u_input, params);
        k4 = quad_dynamics(xstate + dt*k3, u_input, params);
        xstate = xstate + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
        X(:,k) = xstate; Ref(:,k) = [p_ref; v_ref]; Urot(:,k) = f_rotors;
    end

    figure('Name','Validation - 3D Trajectory'); hold on; grid on; plot3(Ref(1,:),Ref(2,:),Ref(3,:),'--','LineWidth',1.2); plot3(X(1,:),X(2,:),X(3,:),'LineWidth',1.4); xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); legend('Ref','Actual'); title('Validation: Reference vs Actual');
    figure('Name','Validation - Position errors'); subplot(3,1,1); plot(time, Ref(1,:)-X(1,:)); ylabel('ex [m]'); grid on; subplot(3,1,2); plot(time, Ref(2,:)-X(2,:)); ylabel('ey [m]'); grid on; subplot(3,1,3); plot(time, Ref(3,:)-X(3,:)); ylabel('ez [m]'); grid on; xlabel('Time [s]');
    figure('Name','Validation - Rotor thrusts'); plot(time, Urot'); legend('f1','f2','f3','f4'); xlabel('Time [s]'); ylabel('Thrust [N]'); grid on;

    fprintf('Validation finished. Final pos error norm = %.6f m\n', norm(Ref(1:3,end) - X(1:3,end)));
end
