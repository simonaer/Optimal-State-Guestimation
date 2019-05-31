function C = C_lin(mu)
    x = mu(1);
    y = mu(2);
    theta = mu(3);
    mx = mu(4);
    my = mu(5);
    C = [(x-mx)/sqrt((x-mx)^2+(y-my)^2)   (y-my)/sqrt((x-mx)^2+(y-my)^2)    0   -(x-mx)/sqrt((x-mx)^2+(y-my)^2)   -(y-my)/sqrt((x-mx)^2+(y-my)^2)];
end