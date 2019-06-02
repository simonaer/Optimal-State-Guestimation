function y = G_NL(x_curr)
    x = x_curr(1);
    y = x_curr(2);
    theta = x_curr(3);
    mx = x_curr(4);
    my = x_curr(5);
    y = [x;
        y;
        theta;
        sqrt((x-mx)^2+(y-my)^2)];
end