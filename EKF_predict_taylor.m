function [mu_predict, sigma_predict] = EKF_predict_taylor(mu,sigma,dt_ctrl,u)
    %predict given a vector of control input (2xt) and dt per control input
    for i = 1:1:size(u,2)
        mu_predict = F_taylor(mu,u(:,i), dt_ctrl);
        A = A_taylor(mu,u(:,i),dt_ctrl);
        sigma_predict = A*sigma*A';
        mu = mu_predict;
        sigma = sigma_predict;
    end
end