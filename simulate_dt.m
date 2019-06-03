function [trajectory] = simulate_dt(xi, u, dt_ctrl, dt_simu)
    %Simulate for specified duration, with euler integration at time intervals
    %of dt_ctrl. Initial state give by xi (5x1), Control input is 2xT, with
    %input specified at each time interval.
    %dt_simu must be divisible by dt_ctrl
    
    trajectory = [];
    
    x = xi;
    

    for i = 1:1:size(u,2)
        %simulate for dt and record trajectory history
        for j = 1:1:dt_ctrl/dt_simu
            rates = EOM(x,u(:,i));
            x = x + dt_simu*rates;
            %x(3) = wrapToPi(x(3)); % wrap theta s.t. [-pi, pi]
            trajectory = [trajectory x];
        end
    end
    
end