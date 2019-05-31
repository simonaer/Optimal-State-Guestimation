%% AA273 Project
clear, close all

% time scale
t0 = 0;
tMax = 50;
dt = 0.5; % simulation time step 50 Hz

T_simu = t0:dt:tMax;

% Initial starting position
mx = 0; %target x position
my = 500; %target y position
xt = [10, 10, -pi/5, mx, my]'; %[px, py, th, mx, my]
ut = [20 0]';

%% Init data storage
X = [];
U = [];
%% simulation
for iter = 1:length(T_simu)
    % MPC for control for next 1/update_freq seconds, how to best reduce
    % the current sigma for target position.
    [ut, extra_out] = aircraftMPC(dt, xt, ut, false);

    rates = EOM(xt,ut);
    xt = xt + rates*dt;
    
    X = [X xt];
    U = [U ut];
end

%% Visualize and Plot
figure
plot(X(1,:), X(2,:))