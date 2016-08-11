% Question 3, part b)
close all; clear; clc;

% Target motion
% T = 1000;  % Time for full run
T = 30; % Time for first leg
dt = 1; % Timestep, 1 Hz
tvec = 0:dt:T; % Time vector
x0_track = [0, 0, 0]; % Starting point for tracking robot
x_track = zeros(T+1,3); % State vector for tracking robot
x_track(1,:) = x0_track;% Starting point for track robot
x_target = zeros(T+1,2); % State vector for target robot
v = 2; % Speed of target in m/s

%Tracker motion
b_scale = 0.1; % scaling noise and nearest distance for q3 b)
% XX,YY cov = 1; theta cov = 1°
Re = b_scale*[1 0 0; 0 1 0; 0 0 1*pi/180];
[ReV, Rev] = eig(Re); % Get eigenvectors/values
tooClose = b_scale*20; %closest the tracker can get to target
kP = 1.0;
v_track = zeros(T+1, 1);

% Target motion and instantaneous occupancy grid map
for t = 2:T+1
    % Target model (provided), do not change
    x_target(t,:) =  [v*t, v*cos(t)];
    
    % Tracking model
    %err = randn(1,3)*[0.5, 0.5, deg2rad(1)]; % 0.25 x,y cov; 1 deg
    e = ReV*sqrt(Rev)*randn(3,1); %  disturbance
    x_des = x_target(t-1,:) - x_track(t-1,1:2); %dx, dy
    ang_des = atan2(x_des(2),x_des(1)); %desired
    ang_err = mod(ang_des - x_track(t-1,3) + pi, 2*pi) - pi; %dtheta
    xdist = sqrt(x_des(1)*x_des(1) + x_des(2)*x_des(2));
    v_track(t) = kP*v*(xdist-tooClose)/tooClose; % match target speed v, stop if too close
    phi_track = ang_err*kP;
    if (xdist < tooClose) 
        v_track(t) = 0; 
        phi_track = 0;
    end
    % Motion
    x_track(t,1) = x_track(t-1,1) + v_track(t)*cos(x_track(t-1,3))*dt + e(1);
    x_track(t,2) = x_track(t-1,2) + v_track(t)*sin(x_track(t-1,3))*dt + e(2);
    x_track(t,3) = x_track(t-1,3) + phi_track*dt + e(3);
 
end

% Plotting scene 
figure(1); clf; subplot(2,1,1); hold on;
plot(x_track(:,1), x_track(:,2), 'b');
plot(x_target(:,1), x_target(:,2), 'r');
title('Question 3: b)');
legend('Tracker', 'Target');
axis equal
hold off;
subplot(2,1,2); hold on;
v_t = [v_track(:).*cos(x_track(:,3)),v_track(:).*sin(x_track(:,3))];
q = quiver(x_track(:,1),x_track(:,2), v_t(:,1), v_t(:,2));
plot(x_target(:,1), x_target(:,2), 'r');
legend('Tracker heading', 'Target');
set(q, 'AutoScaleFactor', 0.3);
axis equal
drawnow;