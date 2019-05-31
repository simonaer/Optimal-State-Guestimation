function [mu,sigma]=EKF_update(mu_predict,sigma_predict,R,y)
C = C_lin(mu_predict);
yhat = G_NL(mu_predict);
mu = mu_predict+sigma_predict*C'*inv(C*sigma_predict*C'+R)*(y-yhat);
sigma = sigma_predict - sigma_predict*C'*inv(C*sigma_predict*C'+R)*C*sigma_predict;
end