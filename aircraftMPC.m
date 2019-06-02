function [u1, extra_out] = aircraftMPC(dt, x0, u0, extra_in)
% goal: calculate the optimal control given the current state and control
% input: time step: dt (ctrl frequency);
%        current state x0 (5x1);
%        current control u0 (2x1);
%        extra_in
% output: optimal control from t0 to t0 + Dt: u1 (2 x dt_iter/dt_ctrl)

n = 3; %dimension of state
m = 2; %dimension of control
u_min = [10, -1/3]';
u_max = [30,  1/3]';
  
T = 3; % planning horizon
N = T/dt; % planning steps = length of control vector

X_offset = x0(4:5);
X_offset = repmat(X_offset, 1, N);

%% Optimization
W_trace = 10; % weight of each objectives
obj_trace = @(U) MPC_predict(extra_in.mu, extra_in.sigma, dt, extra_in.dt_iter, reshape(U, 2, N), extra_in.R);
W_dist = 0.01;
obj_dist = @(U) MPC_dist(x0, dt, N, reshape(U, 2, N));
W_ctrl = 0; %0.1;
obj_ctrl = @(U) MPC_ctrl(u0, reshape(U, 2, N));
J = @(U) W_trace*obj_trace(U) + W_dist* obj_dist(U) + W_ctrl*obj_ctrl(U);
Aineq = [];
bineq = [];
Aeq = [];
beq = [];
lb = repmat(u_min', 1, N); % reshape to 1 x 2N
ub = repmat(u_max', 1, N); % reshape to 1 x 2N
U0 = repmat(u0', 1, N); % reshape to 1 x 2N
[Uf, Jf] = fmincon(J, U0, Aineq, bineq, Aeq, beq, lb, ub);
    
%% Output
U = reshape(Uf, 2, N); % reshape to 2 x N
u1 = U(:, 1:extra_in.dt_iter/dt); % extract the first dt_iter/dt ctrls

extra_out.U = U;
extra_out.obj_tot = Jf;
extra_out.obj_trace = W_trace*obj_trace(Uf);
extra_out.obj_dist = W_dist*obj_dist(Uf);
extra_out.obj_ctrl = W_ctrl*obj_ctrl(Uf);

