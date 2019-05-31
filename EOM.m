function rates = EOM(x,u)
%Equation of Motion
%Returns the rates of states given current states and control inputs
%rates, x, u are all column vectors
%rates = [xdot ydot thetadot mxdot mydot]'
%x = [x y theta mx my]'
%u = [v thetadot]'

v = u(1);
thetadot = u(2);
theta = x(3);

rates = [v*cos(theta);
        v*sin(theta);
        thetadot;
        0;
        0];
end