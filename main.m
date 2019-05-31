%% AA273 Project
clear, close all

% Initial starting position
mx = 50; %target x position
my = 50; %target y position
x0 = [0, 0, deg2rad(0), mx, my]'; %[px, py, th, mx, my]
ut = [10, 0]'; % [v, w]
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
mu_predict = F_NL(mu,ut, 1/update_freq);
A = A_lin(mu,ut);
sigma_predict = A*sigma*A';

%% Init data storage
data_state = x0;
data_ctrl = ut;

%% simulation
for iter = 1:1:tMax/update_freq
    
    % update state to propagate with previously determined control input
    % for a duration equal to 1/update_frequency
    [trajectory control] = simulate_dt(data_state(:,end), ut, 1/update_freq, dt);
    [dist,R] = measure_dist(trajectory(1:3,end),trajetory(4:5,end));
    
    % store history
    data_state = [data_state trajectory];
    data_ctrl = [data_ctrl control];
    
    %EKF update step
    C = lin_C(mu_predict);
    yhat = g_NL(mu_predict);
    
    mu(1:5,iter) = mu_predict+sigma_predict*C'*inv(C*sigma_predict*C'+R)*(y(1:5,iter)-yhat);
    sigma(1:5,iter*5-4:iter*5) = sigma_predict - sigma_predict*C'*inv(C*sigma_predict*C'+R)*C*sigma_predict;
    
    % MPC for control for next 1/update_freq seconds, how to best reduce
    % the current sigma for target position.
    [ut, extra_out] = aircraftMPC(dt, xt, ut, extra_in);
    
    %using control determined by MPC, make EKF prediction
    

    A = A_lin(mu(1:5,iter), ut);
    mu_predict = F_NL(mu(1:5,iter),ut, 1/update_freq);
    sigma_predict = A*sigma(1:5,iter*5-4:iter*5)*A'+Q;
    
    
end

%% Visualize and Plot

