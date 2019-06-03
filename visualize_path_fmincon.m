function [] = visualize_path_fmincon(X,MU,SIGMA,UOPT,dt_simu,dt_ctrl,dt_iter,T_simu)

frames_per_iter = dt_iter/dt_simu;
x_position = X(1,:);
y_position = X(2,:);
theta = X(3,:);
%size of drone to appear
drone_length = 8;
drone_height = 8;
timesteps = size(X,2); %number of timesteps
   

%Animation
f = figure('Color', [1 1 1 1], 'Position', [385 70 711 711])

set(f, 'doublebuffer', 'on');
t = 0;  %Set movie time to 0
i = 1;  %Set index of array to start
pause(1);

v = VideoWriter('Simulate_circle.avi');
v.FrameRate = 1/dt_simu;
dt = dt_simu;

while i<=timesteps
    %Every dt seconds, draw a new plot showing state of aircraft
    %rotate airplane by rotation
    iter = ceil((i-1)/frames_per_iter)+1;
    
    rotation = theta(i)*180/pi;
    drone = imread('drone_img.PNG'); %read image
    %rotate drone
    drone_rotated = imrotate(drone,rotation); 
    Mrot = ~imrotate(true(size(drone)),rotation);
    drone_rotated(Mrot&~imclearborder(Mrot)) = 255;
    RI = imref2d(size(drone_rotated));
    %place drone in the world frame
    RI.XWorldLimits = [x_position(i)-drone_length/2 x_position(i)+drone_length/2];
    RI.YWorldLimits = [y_position(i)-drone_length/2 y_position(i)+drone_length/2];
    %show the plot
    %imshow(drone_rotated, RI)
    imshow(flipud(drone_rotated),RI);
    axis xy;
    axis equal
    %axis display
    axis([-10 80 -120 20])
    title(sprintf('Time = %.2f seconds', T_simu(i)))
    hold on
    %Plot optimal trajectory
    if iter>1
        trajectory = simulate_dt(X(:,(iter-2)*frames_per_iter+1), UOPT(:,:,iter-1), dt_ctrl, dt_simu);
        traj = plot(trajectory(1,:),trajectory(2,:), 'g')
    else
        traj = plot(0,0,'g')
    end
    %plot trail
    trail = plot(X(1,1:i),X(2,1:i), 'r')
    %Plot actual position of target
    sc_target = scatter(X(4,i),X(5,i),'r')
    sc_target.Marker = 'x';
    sc_target.SizeData = 50;
    %Plot mu position
    sc = scatter(MU(4,iter),MU(5,iter), 'b')
    sc.Marker = 'x';
    sc.SizeData = 50;
    %plot sigma ellipse
    P = 0.95;
    theta_ellipse = 0:0.02:2*pi;
    r_circle = sqrt(-2*log(1-P));
    x_circle = r_circle*cos(theta_ellipse);
    y_circle = r_circle*sin(theta_ellipse);

    for j=1:1:length(x_circle)
        w = [x_circle(j);y_circle(j)];
        ellipse(:,j) = SIGMA(4:5,iter*5-1:iter*5)^0.5*w+MU(4:5,iter);
    end

    ellip =plot(ellipse(1,:),ellipse(2,:), 'b')
    legend([traj trail sc_target sc ellip], {'circle tracking', 'trajectory', 'beacon position', 'mu', 'error ellipse'}, 'Location', 'SouthWest')
    grid minor
    
    
    hold off
    
    
    %set the size of the figure so that frames are all same dimensions,
    %otherwise video wont make
    set(gcf, 'Position', [385 70 711 711])
    
    drawnow;
    
    
    
    M(i) = getframe(gcf);
    pause(0.1)
    t = t+dt;
    i=i+1;
end

open(v);
writeVideo(v,M);
close(v)