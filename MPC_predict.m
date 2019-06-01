function [matrix_trace] = MPC_predict(mu,sigma,dt_ctrl,dt_iter,u,R)
    %predict given a vector of control input (2xt) and dt per control input

    time_horizon = size(u,2)*dt_ctrl;
    measurements = time_horizon/dt_iter;
    ctrl_per_iter = dt_iter/dt_ctrl;
    for i = 1:1:measurements

        [mu_predict, sigma_predict] = EKF_predict(mu,sigma,dt_ctrl,u(:,i*ctrl_per_iter-(ctrl_per_iter-1):i*ctrl_per_iter));
        mu = mu_predict;
        C = C_lin(mu_predict);
        sigma = sigma_predict - sigma_predict*C'*inv(C*sigma_predict*C'+R)*C*sigma_predict;
        
    end
    matrix_trace = trace(sigma);
end