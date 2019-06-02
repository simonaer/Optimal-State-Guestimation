function val = MPC_ctrl(u0, U)
% input: x0 = 2x1 = current control
%        U = 2xN
% output: val of control jerks
val = 0;
U = [u0 U];
for i = 2:size(U,2)
    v_diff = square((U(1,i) - U(1,i-1))/20); % range = [-1 1]
    w_diff = square((U(2,i) - U(2,i-1))/(2/3));
    val = val + 0.5*v_diff + w_diff;
end