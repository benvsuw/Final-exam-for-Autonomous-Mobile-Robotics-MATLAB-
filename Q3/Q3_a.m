% Target Tracking 
close all; clear; clc;

% Read in map
I = imread('trackingmap.jpg'); % Map defined as 1 pixel = 1 m
image(I);
map = im2bw(I, 0.4); % Convert to 0-1 image
map = flipud(map)'; % Convert to 0 free, 1 occupied and flip.
[M,N]= size(map); % Map size in m

% Target tour in m
tour = [ 28   654; 35   501; 70   435; 79   174; 178   134;
   279   148; 308   273; 338   421; 390   551; 411   652;
   613   657; 705   565; 733   419; 856   330; 957   330;
   969   231; 940   198; 893   304; 703   291; 578   151;
   458   224; 352   278; 296   242; 258   153; 108   167;
   103   414; 112   515; 148   612; 91   642; 28   654];

% Target motion
% T = 1000;  % Time for full run
T = 30; % Time for first leg
dt = 1; % Timestep
tvec = 0:dt:T; % Time vector
x0_track = [100, 650, pi]; % Starting point for tracking robot
x_target = zeros(T+1,2); % State vector for target robot
x_target(1,:) = tour(1,:); % Starting point for target robot
Ntour = 1; % Tour segment counter
v = 5; % Speed of target in m/s

%Tracker motion
% XX,YY cov = 1; theta cov = 1°
Re = [1 0 0; 0 1 0; 0 0 1*pi/180];
[ReV, Rev] = eig(Re); % Get eigenvectors/values
x_track = zeros(T+1,3); % State vector for tracking robot
x_track(1,:) = x0_track;% Starting point for track robot
tooClose = 20; %closest the tracker can get to target
kP = 1.0;

% Target motion and instantaneous occupancy grid map
for t = 2:T+1
    % Target model (provided), do not change
    motion_dir = tour(Ntour+1,:)-tour(Ntour,:);
    motion_dir = motion_dir/norm(motion_dir);
    x_target(t,:) =  x_target(t-1,:) + motion_dir*v*dt;
    if (norm(x_target(t,:)-tour(Ntour+1,:)) < v*dt)
        Ntour = Ntour+1;
        if (Ntour == length(tour(:,1)))
            break;
        end
    end
    
    % Tracking model
    %err = randn(1,3)*[0.5, 0.5, deg2rad(1)]; % 0.25 x,y cov; 1 deg
    e = ReV*sqrt(Rev)*randn(3,1); %  disturbance
    % Controller. To t state from t-1 state
    x_des = x_target(t-1,:) - x_track(t-1,1:2); %dx, dy
    ang_des = atan2(x_des(2),x_des(1)); %desired
    ang_err = mod(ang_des - x_track(t-1,3) + pi, 2*pi) - pi; %dtheta
    xdist = sqrt(x_des(1)*x_des(1) + x_des(2)*x_des(2));
    v_track = kP*v*(xdist-tooClose)/tooClose; % match target speed v, stop if too close
    phi_track = ang_err*kP;
    if (xdist < tooClose) 
        v_track = 0; 
        phi_track = 0;
    end
    % Motion model is simplified because [v u] -> r[w1 w2] -> [v u]
    x_track(t,1) = x_track(t-1,1) + v_track*cos(x_track(t-1,3))*dt + e(1);
    x_track(t,2) = x_track(t-1,2) + v_track*sin(x_track(t-1,3))*dt + e(2);
    x_track(t,3) = x_track(t-1,3) + phi_track*dt + e(3);
    
    
    % Plotting scene (slow, comment out if not needed)
    figure(1); clf; hold on;
    colormap('gray');
    imagesc(1-map');
    plot(tour(:,1),tour(:,2),'w')
    drawbot(x_track(t,1),x_track(t,2),x_track(t,3),10,1); % Draw the tracking robot
    targetbot = drawbot(x_target(t,1),x_target(t,2),atan2(motion_dir(2), motion_dir(1)),10,1);
    axis equal
    drawnow;

    % Plotting map (slow, comment out if not needed)
    figure(2); clf; hold on;
    colormap('gray');
    curmap = addbot2map(map, targetbot);
    imagesc(1-curmap');
    axis equal
    drawnow;
end

