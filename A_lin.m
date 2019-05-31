function A = A_lin(mu,u,dt)
    x = mu(1);
    y = mu(2);
    theta = mu(3);
    mx = mu(4);
    my = mu(5);
    v = u(1);
    w = u(2);
    A = [1  0   -v*sin(theta)*dt 0 0;
        0   1   v*cos(theta)*dt  0 0;
        0   0   1   0   0;
        0   0   0   1   0;
        0   0   0   0   1];