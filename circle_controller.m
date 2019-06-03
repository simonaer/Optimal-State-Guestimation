function ut = circle_controller(x,mu,dt_iter, dt_ctrl)
    ctrl_per_iter = dt_iter/dt_ctrl;
    w_target = 0.2;
    v_target = 10;
    period = 2*pi/w_target;
    circum = period*v_target;
    radius = circum/2/pi;
    dist = sqrt((x(1)-mu(1))^2+(x(2)-mu(2))^2);
    phi = atan2(x(2)-mu(2), x(1)-mu(1));
    
    if abs(radius-dist)>20
        theta_target =  phi-pi;
        thetadot = (theta_target-x(3))/dt_iter;
        thetadot = sign(thetadot)*min(abs(thetadot), 1/3);
        ut = [ones(1,ctrl_per_iter)*10;
            ones(1,ctrl_per_iter)*thetadot];
    else
        theta_target =  phi-pi;
        thetadot = (theta_target-x(3))/dt_iter;
        thetadot = sign(thetadot)*min(abs(thetadot)+(dist-radius)/30, 1/3);
        ut = [ones(1,ctrl_per_iter)*10;
            ones(1,ctrl_per_iter)*thetadot];
    end
        
end