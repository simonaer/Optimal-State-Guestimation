function [matrix_trace] = aircraftMPC_MS_predict(mu,sigma,dt_ctrl,dt_iter,x,R)
    %predict given a vector of control input (2xt) and dt per control input
    gridN = length(x)/5;
    u = [x(3*gridN+1:4*gridN)';
        x(4*gridN+1:5*gridN)'];
    time_horizon = size(u,2)*dt_ctrl;
    measurements = time_horizon/dt_iter;
    ctrl_per_iter = dt_iter/dt_ctrl;
    
    for i = 1:1:measurements

        [mu_predict, sigma_predict] = EKF_predict(mu,sigma,dt_ctrl,u(:,i*ctrl_per_iter-(ctrl_per_iter-1):i*ctrl_per_iter));
        mu = mu_predict;
        C = C_lin(mu_predict);
        [void, R] = measure_dist(mu_predict(1:3),mu_predict(4:5));
        sigma = sigma_predict - sigma_predict*C'*inv(C*sigma_predict*C'+R)*C*sigma_predict;
        
    end
    matrix_trace = trace(sigma);
    
end