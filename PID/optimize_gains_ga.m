clear; close all; clc;

controller_type = 'pid';

% GA variables layout for 'pid'
% x = [Kp_pos_x, Kp_pos_y, Kp_pos_z,
%      Kd_pos_x, Kd_pos_y, Kd_pos_z,
%      Ki_pos_x, Ki_pos_y, Ki_pos_z,
%      Kp_att_x, Kp_att_y, Kp_att_z,
%      Kd_att_x, Kd_att_y, Kd_att_z];

nvars = 15;

lb = [ 0, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0,    0, 0, 0 ];
ub = [10,10,20,   10,10,10,   1, 1, 2,   300,300,150, 10,10,10];

options = optimoptions('ga',...
    'PopulationSize', 60, ...
    'MaxGenerations', 40, ...
    'Display','iter', ...
    'UseParallel', false, ... 
    'PlotFcn', {@gaplotbestf});

objfun = @(x) sim_cost_for_gains(x, controller_type);

rng(1,'twister');
[xbest, fbest, exitflag, output] = ga(objfun, nvars, [],[],[],[], lb, ub, [], options);

fprintf('GA done. Best cost = %.4f\n', fbest);
disp('Best gains:');
disp(xbest);

