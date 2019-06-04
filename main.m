%% AA273 Project
clear
%close all

% time scale
t0 = 0;
tMax = 20;
dt_simu = 0.02; % simulation time step 50 Hz
dt_iter = 1.0; % main loop update time step 1Hz
dt_ctrl = 0.2; % control updating time step;
T_iter = t0:dt_iter:tMax;
T_ctrl = t0:dt_ctrl:tMax;
T_simu = t0:dt_simu:tMax;

% Initial starting position
mx = 10; %target x position
my = -50; %target y position
xt = [0, 0, deg2rad(0), mx, my]'; %[px, py, th, mx, my]
ut = [10*ones(1,dt_iter/dt_ctrl);
      0*ones(1,dt_iter/dt_ctrl)]; % [v, w]
%Constraints
v_min = 10; %minimum speed is 10 m/s
w_max = 1/3; %maximum 0.333 rad/s in turning

%EKF initialization
mu = [xt(1); xt(2); xt(3); 40; -30];
sigma = [0 0 0 0 0;
        0 0 0 0 0;
        0 0 0 0 0;
        0 0 0 50 0;
        0 0 0 0 50];
%EKF prediction
[mu_predict, sigma_predict] = EKF_predict(mu, sigma, dt_ctrl, ut);

% Init data storage
X = xt;
U = ut;
Y = [xt(1:3);0];
MU = mu;
SIGMA = sigma;
UOPT = [];
dist = [0];
% simulation
for iter = 2:length(T_iter)
    
    t = T_iter(iter);
    xt = X(1:5,end);
    % update state to propagate with previously determined control input
    % for a duration equal to 1/update_frequency
    [trajectory] = simulate_dt(xt, ut, dt_ctrl, dt_simu);
    [y,R] = measure_dist(trajectory(1:3,end),trajectory(4:5,end));
    
    %EKF update step
    [mu,sigma] = EKF_update(mu_predict,sigma_predict,R,y);
    
    % MPC for control for next 1/update_freq seconds, how to best reduce
    % the current sigma for target position.
    extra_in.mu = mu;
    extra_in.sigma = sigma;
    extra_in.dt_iter = dt_iter;
    extra_in.R = R;
    %uncomment next two lines for MPC
    [ut, extra_out] = aircraftMPC(dt_ctrl, mu, ut(:,1), extra_in); % TODO: should we input xt or mu to optimizer?
    full_ut_opt = extra_out.U;
    
    %uncomment next two lines for simple circle tracking
    %ut = circle_controller(X(:,end),mu,dt_iter, dt_ctrl);
    %full_ut_opt = ut;
    %using control determined by MPC, make EKF prediction
    
    [mu_predict, sigma_predict] = EKF_predict(mu, sigma, dt_ctrl, ut);
       
    % store history
    X = [X trajectory];
    U = [U ut];
    UOPT(:,:,iter) = full_ut_opt;
    MU = [MU mu];
    SIGMA = [SIGMA sigma];
    Y = [Y y];
    distance = sqrt((X(2,end)-mu(5))^2+(X(1,end)-mu(4))^2);
    dist = [dist distance];
end

%% Visualize and Plot
figure(1)
hold on
grid on
plot(X(1,:), X(2,:))
plot(MU(1,:), MU(2,:))
legend('X','mu')

figure(2)
hold on
grid on
plot(X(1,:), X(2,:))
plot(MU(4,:), MU(5,:))
legend('X','target')

for i = 1:1:size(MU,2)
    SIGMA_TRACE(i) = trace(SIGMA(1:5,i*5-4:i*5));
end
figure
plot(T_iter,SIGMA_TRACE)
figure
plot(T_iter, dist)

%%
%visualize_path_fmincon(X,MU,SIGMA,UOPT,dt_simu,dt_ctrl,dt_iter,T_simu)