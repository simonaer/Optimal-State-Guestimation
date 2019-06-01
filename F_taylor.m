function x_next = F_taylor(x_curr, u, dt)
    x = x_curr(1);
    y = x_curr(2);
    theta = x_curr(3);
    mx = x_curr(4);
    my = x_curr(5);
    v = u(1);
    w = u(2);
    
%     % Theta wrapper --> \theta = [0, 2*pi]
%     theta = (theta^2)^0.5;
%     theta = mod(theta, 2*pi);
    
    mycos = 1-theta^2/2;
    mysin = theta;
    x_next = [x+dt*v*mycos;
              y+dt*v*mysin;
              theta+dt*w;
              mx;
              my];
end