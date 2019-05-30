function [trajectory, control] = simulate_dt(xi, u, duration, dt)
    %Simulate for specified duration, with euler integration at time intervals
    %of dt. Initial state give by xi (5x1), and hold control input of u for
    %duration of simulation. u is 2x1.
    
    trajectory = [];
    control = [];
    x = xi;
    t = dt;
    while t<=duration
        %simulate for dt and record trajectory history
        rates = EOM(x,u);
        x = x + dt*rates;
        trajectory = [trajectory x];
        control = [control u];
        t = t+dt;
    end
    
end