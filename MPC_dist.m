function val = MPC_dist(x0, dt, N, U)
% input: x0 = 5x1 = current state
%        dt = control freq
%        N = optimization horizon
%        U = 2xN
% output: val of distance between current state and the beacon at each step
x = x0;
val = 0;
for i = 1:N
    rates = EOM(x,U(:,i));
    x = x + rates * dt;
    val = val + norm(x(1:2) - x(4:5));
end