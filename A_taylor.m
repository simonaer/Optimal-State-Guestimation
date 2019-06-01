function A = A_taylor(mu,u,dt)
    x = mu(1);
    y = mu(2);
    theta = mu(3);
    mx = mu(4);
    my = mu(5);
    v = u(1);
    w = u(2);
    
%     % Theta wrapper --> \theta = [0, 2*pi]
%     if(abs(theta) > 2*pi) 
%         theta = mod(theta, 2*pi);
%     end
    
    mycos = 1-theta^2/2;
    mysin = theta;
    
    A = [1  0   -v*mysin*dt 0 0;
        0   1   v*mycos*dt  0 0;
        0   0   1   0   0;
        0   0   0   1   0;
        0   0   0   0   1];