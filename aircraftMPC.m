function [u1, extra_out] = aircraftMPC(dt, x0, u0, extra_in)
% goal: calculate the optimal control given the current state and control
% input: time step dt; current state x0; current control u0;
% output: optimal control from t0 to t0 + Dt

n = 3; %dimension of state
m = 2; %dimension of control

%% Define constraints
u_min = [10, -1/3]';
u_max = [30, 1/3]';

%% Dynamics
At =  A_lin(x0,u0,dt);
At = At(1:3, 1:3);
Bt = [cos(x0(3)), 0; sin(x0(3)), 0; 0, 1] * dt;
  
T = 1; % planning horizon
N = T/dt; % planning steps

X_offset = x0(4:5);
X_offset = repmat(X_offset, 1, N);

%% Optimization
cvx_precision LOW
cvx_begin quiet
    variables X(n,N+1) U(m, N)

    % inequality constraints
%     max(X') <= x_max'; min(X') >= x_min';
    max(U') <= u_max'; min(U') >= u_min';
    
    % equality (dynamic) constraints
    X(:,1) == x0(1:3); 
    X(:,2:N+1) == At*X(:,1:N) + Bt*U;

    % calculate objectives from EKF update step
    sig_trace = MPC_predict(extra_in.mu, extra_in.sigma, dt, extra_in.dt_iter, U ,extra_in.R);
    dist_offset = sum(sum_square(X(1:2,2:N+1) - X_offset));
%     ctrl_effort = sum(sum_square(0*(U(1,2:N) - U(1,1:N-1))));
    
    % objective
    minimize(sig_trace + dist_offset);
    
cvx_end

%% Output
u1 = U(:,extra_in.dt_iter/dt); 

extra_out.U = U;
extra_out.X = X;
extra_out.obj = cvx_optval;

