function A = A_lin(mu,u)
    x = mu(1);
    y = mu(2);
    theta = mu(3);
    mx = mu(4);
    my = mu(5);
    v = u(1);
    w = u(2);
    A = [0  0   -v*sin(theta) 0 0;
        0   0   v*cos(theta)  0 0;
        0   0   0   0   0;
        0   0   0   0   0;
        0   0   0   0   0];