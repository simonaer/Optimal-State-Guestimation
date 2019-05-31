%% AA273 Project
clear, close all

% time scale
t0 = 0;
tMax = 50;
dt_simu = 0.02; % simulation time step 50 Hz
dt_iter = 1.0; % main loop update time step 1Hz
dt_ctrl = dt_simu; % control updating time step;
T_iter = t0:dt_iter:tMax;
T_ctrl = t0:dt_ctrl:tMax;
T_simu = t0:dt_simu:tMax;

% Initial starting position
mx = 0; %target x position
my = 15; %target y position
xt = [0, 0, 0, mx, my]'; %[px, py, th, mx, my]
ut = [10*ones(1,dt_iter/dt_simu);
      1/3*ones(1,dt_iter/dt_simu)]; % [v, w]
%Constraints
v_min = 10; %minimum speed is 10 m/s
w_max = 1/3; %maximum 0.333 rad/s in turning

%EKF initialization
mu = [0;0;0;-10;-10];
sigma = [0 0 0 0 0;
        0 0 0 0 0;
        0 0 0 0 0;
        0 0 0 50 0;
        0 0 0 0 50];
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
    [mu,sigma]=EKF_update(mu_predict,sigma_predict,R,y);
    
    % MPC for control for next 1/update_freq seconds, how to best reduce
    % the current sigma for target position.
    %[ut, extra_out] = aircraftMPC(dt_simu, xt, ut, extra_in);
    xt = X(1:5,end);
    ut = [10*ones(1,dt_iter/dt_simu);
        1/3*ones(1,dt_iter/dt_simu)];
    %using control determined by MPC, make EKF prediction
    [mu_predict, sigma_predict] = EKF_predict(mu, sigma, dt_ctrl, ut);
       
    % store history
    X = [X trajectory];
    U = [U ut];
    MU = [MU mu];
    SIGMA = [SIGMA sigma];
    Y = [Y y];
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
