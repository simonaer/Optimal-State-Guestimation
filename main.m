%% AA273 Project
clear, close all

% time scale
t0 = 0;
tMax = 20;
dt_simu = 0.02; % simulation time step 50 Hz
update_freq_Hz = 1; %1 measurement per second
dt_iter = 1/update_freq_Hz; % main loop update time step 1Hz
dt_ctrl = dt_simu; % control updating time step;
T_iter = t0:dt_iter:tMax;
T_ctrl = t0:dt_ctrl:tMax;
T_simu = t0:dt_simu:tMax;

% Initial starting position
mx = 50; %target x position
my = 50; %target y position
xt = [0, 0, 0, mx, my]'; %[px, py, th, mx, my]
ut = [10*ones(1,dt_iter/dt_simu);
      zeros(1,dt_iter/dt_simu)]; % [v, w]
%Constraints
v_min = 10; %minimum speed is 10 m/s
w_max = 1/3; %maximum 0.333 rad/s in turning



%EKF initialization
mu = [0;0;0;30;30];
sigma = 50*eye(5);
%EKF prediction
[mu_predict, sigma_predict] = EKF_predict(mu, sigma, dt_ctrl, ut);



%% Init data storage
X = xt;
U = ut;
Y = [0];
MU = mu;
SIGMA = sigma;
%% simulation
for iter = 2:length(T_iter)
    
    t = T_iter(iter);
    
    % update state to propagate with previously determined control input
    % for a duration equal to 1/update_frequency
    [trajectory] = simulate_dt(xt, ut, dt_ctrl);
    [y,R] = measure_dist(trajectory(1:3,end),trajectory(4:5,end));
    
    
    %EKF update step
    %Add update function Here to replace next 4 lines
    C = C_lin(mu_predict);
    yhat = G_NL(mu_predict);
    mu = mu_predict+sigma_predict*C'*inv(C*sigma_predict*C'+R)*(y-yhat);
    sigma = sigma_predict - sigma_predict*C'*inv(C*sigma_predict*C'+R)*C*sigma_predict;
    
    % store history
    X = [X trajectory];
    U = [U ut];
    MU = [MU mu];
    SIGMA = [SIGMA sigma];
    Y = [Y y];
    % MPC for control for next 1/update_freq seconds, how to best reduce
    % the current sigma for target position.
    %[ut, extra_out] = aircraftMPC(dt_simu, xt, ut, extra_in);
    xt = X(1:5,end);
    ut = [10*ones(1,dt_iter/dt_simu);
            zeros(1,dt_iter/dt_simu)];
    %using control determined by MPC, make EKF prediction
    [mu_predict, sigma_predict] = EKF_predict(mu, sigma, dt_ctrl, ut);
       
    
end

%% Visualize and Plot
figure
plot(T_simu,X(1,:), T_iter, MU(1,:))
figure
plot(T_simu,X(2,:), T_iter, MU(2,:))
figure
plot(T_simu,X(3,:), T_iter, MU(3,:))
figure
plot(T_simu,X(4,:), T_iter, MU(4,:))
figure
plot(T_simu,X(5,:), T_iter, MU(5,:))
