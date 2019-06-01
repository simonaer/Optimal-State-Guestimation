%% AA273 Project
clear, close all

% time scale
t0 = 0;
tMax = 5;
dt_simu = 0.02; % simulation time step 50 Hz
dt_iter = 1.0; % main loop update time step 1Hz
dt_ctrl = 0.5; % control updating time step;
T_iter = t0:dt_iter:tMax;
T_ctrl = t0:dt_ctrl:tMax;
T_simu = t0:dt_simu:tMax;

% Initial starting position
mx = 0; %target x position
my = 100; %target y position
xt = [10, 10, -deg2rad(30), mx, my]'; %[px, py, th, mx, my]
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
    xt = X(1:5,end);
    % update state to propagate with previously determined control input
    % for a duration equal to 1/update_frequency
    [trajectory] = simulate_dt(xt, ut, dt_ctrl);
    [y,R] = measure_dist(trajectory(1:3,end),trajectory(4:5,end));
    
    %EKF update step
    [mu,sigma] = EKF_update(mu_predict,sigma_predict,R,y);
    
    % MPC for control for next 1/update_freq seconds, how to best reduce
    % the current sigma for target position.
    extra_in.mu = mu;
    extra_in.sigma = sigma;
    extra_in.dt_iter = dt_iter;
    extra_in.R = R;
    [ut, extra_out] = aircraftMPC(dt_ctrl, xt, ut(:,1), extra_in);
%     xt = X(1:5,end);
%     ut = [10*ones(1,dt_iter/dt_simu);
%         1/3*ones(1,dt_iter/dt_simu)];
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
figure(1)
hold on
grid on
plot(X(1,:), X(2,:))
plot(MU(1,:), MU(2,:))
legend('X','mu')

