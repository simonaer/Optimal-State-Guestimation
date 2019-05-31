function [trajectory] = simulate_dt(xi, u, dt_ctrl)
    %Simulate for specified duration, with euler integration at time intervals
    %of dt_ctrl. Initial state give by xi (5x1), Control input is 2xT, with
    %input specified at each time interval.
    
    trajectory = [];
    
    x = xi;

    for i = 1:1:size(u,2)
        %simulate for dt and record trajectory history
        rates = EOM(x,u(:,i));
        x = x + dt_ctrl*rates;
        trajectory = [trajectory x];
    end
    
end