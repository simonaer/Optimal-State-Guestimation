function [y,R]=measure_dist(x,m)
a0=1;%m^2
a1=1;%m^2
a2=0.0024;%m^2

r=norm(m-x(1:2));%true distance 
sig_dis = a2*(r-a1)^2 + a0; %variance_dis
R = [0.01 0 0 0;
    0 0.01 0 0;
    0 0 0.01 0;
    0 0 0 sig_dis];
y=[x(1);
   x(2);
   x(3);
   norm(m-x(1:2))+mvnrnd(0,sig_dis)];% distance measurement
end

