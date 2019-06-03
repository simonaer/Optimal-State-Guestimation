function [ c, ceq ] = aircraftMPC_MS_constraints( x0 )
    global gridN
    global control_dt
    global simu_dt
    % No nonlinear inequality constraint needed
    c = [];
    % Get the states / inputs out of the vector
    x = x0(1             :gridN);
    y = x0(1 + gridN     :gridN * 2);
    theta = x0(1 + gridN * 2 : gridN * 3);
    u = x0(1 + gridN * 3 : gridN * 4);
    w = x0(1 + gridN * 4 : gridN * 5); %thetadot for now
    % Constrain initial position and velocity to be current location
    ceq = [x0(1); y(1); theta(1)];
    for i = 1 : length(x) - 1
        % The state at the beginning of the time interval
        x_i = [x0(i); y(i);theta(i)];
        % What the state should be at the start of the next time interval
        x_n = [x0(i+1); y(i+1);theta(i+1)];

        
        % The end state of the time interval using EOM
        trajectory = simulate_dt([x_i;0;0],[u(i);w(i)],control_dt,simu_dt);
        % Constrain the end state of the current time interval to be
        % equal to the starting state of the next time interval
        xend = trajectory(1:3,end);
        ceq = [ceq ; x_n - xend];
    end
    % free end condition
    
end