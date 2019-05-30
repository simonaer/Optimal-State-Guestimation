%% AA273 Project
clear, close all

% Initial starting position
x0 = [0, 0, deg2rad(0)]'; %[px, py, th]
u0 = [0, 0]'; % [v, w]
% time scale
t0 = 0;
tMax = 20;
dt = 0.02;
tau = t0:dt:tMax;

%% Init data storage
data_state = zeros(3, length(tau) + 1); data_state(:, 1) = x0;
data_ctrl = zeros(2, length(tau));

%% simulation
for iter = 1:length(tau)
    t = tau(iter);
    
    % MPC for control
    [ut, extra_out] = aircraftMPC(dt, xt, ut, extra_in);
    
    % state updates
    xt = updateState(dt, xt, ut);
    
    % store history
    data_state(:, iter+1) = xt;
    data_ctrl(:, iter) = ut;
end