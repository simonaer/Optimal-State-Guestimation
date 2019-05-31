%% AA273 Project
clear, close all

% Initial starting position
mx = 50; %target x position
my = 50; %target y position
x0 = [0, 0, deg2rad(0), mx, my]'; %[px, py, th, mx, my]
u0 = [10, 0]'; % [v, w]
%Constraints
v_min = 10; %minimum speed is 10 m/s
w_max = 1/3; %maximum 0.333 rad/s in turning
% time scale
t0 = 0;
tMax = 20;
dt = 0.02;
update_freq = 1; %1 measurement per second
tau = t0:dt:tMax;

%EKF initialization
mu = [0;0;0;0;0];
sigma = 10*eye(5);
%EKF prediction
mu_predict = F_NL(mu,u0);
sigma_predict = 

%% Init data storage
data_state = x0;
data_ctrl = u0;

%% simulation
for iter = 1:1:tMax/update_freq
    
    % update state to propagate with previously determined control input
    % for a duration equal to 1/update_frequency
    [trajecotry control] = simulate_dt(data_state(:,end), ut, 1/update_freq, dt);
    
    % store history
    data_state = [data_state trajetory];
    data_ctrl = [data_ctrl control];
    
    %EKF update step
    
    % MPC for control for next 1/update_freq seconds, how to best reduce
    % the current sigma for target position.
    [ut, extra_out] = aircraftMPC(dt, xt, ut, extra_in);
    
    %using control determined by MPC, make EKF prediction
    
    
    
<<<<<<< HEAD
    % store history
    data_state(:, iter+1) = xt;
    data_ctrl(:, iter) = ut;
end

%% Visualize and Plot
=======
end
>>>>>>> c437474d5b2726e8b09ff691a3bbca5a714ccadf
