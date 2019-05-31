function x_next = F_NL(x_curr, u, dt)
    x = x_curr(1);
    y = x_curr(2);
    theta = x_curr(3);
    mx = x_curr(4);
    my = x_curr(5);
    v = u(1);
    w = u(2);
    x_next = [x+dt*v*cos(theta);
              y+dt*v*sin(theta);
              theta+dt*w;
              mx;
              my];
end