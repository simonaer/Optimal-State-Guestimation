function y=angle(x,m)
a0=1;%m^2
a1=1;%m^2
a2=0.0024;%m^2
alpha=0.01;%(rad/m)^2

r=norm(m-x(1:2));%true distance 
sig_angle = alpha*(a2*(r-a1)^2 + a0); %variance_angle
y=atan2((m(2)-x(2)),(m(1)-x(1)))+mvnrnd(0,sqrt(sig_angle));% angle measurement
end
