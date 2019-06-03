function [ut] = aircraftMPC_MS(dt_ctrl, dt_simu, dt_iter, mu, u0, sigma, R)
% clear
% close all
% dt_ctrl = 0.2;
% dt_simu = 0.02;
% dt_iter = 1;
% mu = [0;0;0;0;50];
% mx = 0;
% my = 50;
% sigma = [0.01 0 0 0 0;
%         0 0.01 0 0 0;
%         0 0 0.01 0 0;
%         0 0 0 50 0;
%         0 0 0 0 50];
% [void,R] = measure_dist([0;0;0],[mx,my]);
horizon = dt_iter;
global control_dt
global simu_dt
control_dt = 3*dt_ctrl;
simu_dt = dt_simu;

%Constraints
v_min = 10; %minimum speed is 10 m/s
w_max = 1/3; %maximum 0.333 rad/s in turning

%initial mu and sigma

global gridN
gridN = horizon/dt_ctrl;
tic
% Minimize the trace of the covariance at end of time horizon

% The initial parameter guess: gridN positions/angles, gridN velocities,
% gridN w's (a straight line in current direction at 150% of minimum
% velocity
x0 = [linspace(mu(1),mu(1)+1.5*v_min*cos(mu(3))*horizon,gridN)'; linspace(mu(2),mu(2)+1.5*v_min*sin(mu(3))*horizon,gridN)'; mu(3)*ones(gridN,1); v_min*1.5*ones(gridN, 1);zeros(gridN, 1)];
% No linear inequality or equality constraints
A = [];
b = [];
Aeq = [];
Beq = [];
% Lower bound and upper bound of positions and angles is -inf/inf
% Bound velocity between vmin and 5*vmin
lb = [ones(gridN * 3, 1) * -Inf;  ones(gridN, 1)*v_min; ones(gridN, 1) * -w_max];
ub = [ones(gridN * 3, 1) * Inf;   ones(gridN, 1) * 3*v_min; ones(gridN, 1) * w_max];
% Options for fmincon
%options = optimoptions(@fmincon, 'TolFun', 0.000001, 'MaxIter', 200, ...
                       %'MaxFunEvals', 20000, 'Display', 'iter', ...
                       %'DiffMinChange', 0.001, 'Algorithm', 'sqp');
options = optimoptions(@fmincon, 'TolFun', 0.00000001, ...
                       'MaxFunEvals', 20000);
% Solve for the best simulation time + control input
optimal = fmincon(@(x) aircraftMPC_MS_predict(mu,sigma,dt_ctrl,dt_iter,x,R), x0, A, b, Aeq, Beq, lb, ub, ...
              @aircraftMPC_MS_constraints, options);
% Discretize the times
x = optimal(1             :gridN);
y = optimal(1 + gridN     :gridN * 2);
theta = optimal(1 + gridN * 2 : gridN * 3);
u = optimal(1 + gridN * 3 : gridN * 4);
w = optimal(1 + gridN * 4 : gridN * 5);

ut = [u(1:dt_iter/dt_ctrl)';w(1:dt_iter/dt_ctrl)'];
% Make the plots


disp(sprintf('Finished in %f seconds', toc));
end