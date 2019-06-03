function ut = circle_controller(X,mu,dt_iter, dt_ctrl)
    ctrl_per_iter = dt_iter/dt_ctrl;
    w_target = 0.25;
    v_target = 10;
    period = 2*pi/w_target;
    circum = period*v_target;
    radius = circum/2/pi;
    x = X;
    dist = sqrt((x(1)-mu(4))^2+(x(2)-mu(5))^2);
    phi = atan2(x(2)-mu(5), x(1)-mu(4));
    ut = [];
    for i = 1:1:ctrl_per_iter
        
        
        theta_target =  phi-dist/radius*pi/2;
        theta_error = wrapToPi(theta_target-x(3));
        thetadot = sign(theta_error)*1/3;
        
        ut = [ut [10;thetadot]];
        rates = EOM(x,ut(:,end));
        x = x + dt_ctrl*rates;
        dist = sqrt((x(1)-mu(4))^2+(x(2)-mu(5))^2);
        phi = atan2(x(2)-mu(5), x(1)-mu(4));
        
    end
    
        
end