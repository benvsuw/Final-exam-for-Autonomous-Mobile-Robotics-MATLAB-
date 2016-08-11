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
%T = 1000;  % Time for full run
T = 200; % Time for 1/3 - c)
%T = 30; % Time for first leg
dt = 1; % Timestep
tvec = 0:dt:T; % Time vector
x0_track = [100, 650, pi]; % Starting point for tracking robot
x_target = zeros(T+1,2); % State vector for target robot
x_target(1,:) = tour(1,:); % Starting point for target robot
Ntour = 1; % Tour segment counter
v = 5; % Speed of target in m/s

%Tracker motion - a)
% X,Y var = 1; XY cov = 0.5; theta var = 1°
Re = 0.1*[1 0.5 0; 0.5 1 0; 0 0 1*pi/180];
[ReV, Rev] = eig(Re); % Get eigenvectors/values
x_track = zeros(T+1,3); % State vector for tracking robot
x_track(1,:) = x0_track;% Starting point for track robot
tooClose = 45; %closest the tracker can get to target
kP = 1.0;
v_track = zeros(T+1,1); % command signal
phi_track = zeros(T+1,1);% command signal

%Occupancy grid - c)
s_n = 200; % Scan points
s_fov = deg2rad(180); %scan field of view
s_range = 75; %75m = 75px
% Belief map
s_map = 0.5*ones(M,N);
L0 = log(s_map./(1-s_map));
L=L0;
%update rate 1Hz, agrees with model


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
    
    % Tracking model - a)
    %err = randn(1,3)*[0.5, 0.5, deg2rad(1)]; % 0.25 x,y cov; 1 deg
    e = ReV*sqrt(Rev)*randn(3,1); %  disturbance
    % Controller. To t state from t-1 state
    x_des = x_target(t-1,:) - x_track(t-1,1:2); %dx, dy
    ang_des = atan2(x_des(2),x_des(1)); %desired
    ang_err = mod(ang_des - x_track(t-1,3) + pi, 2*pi) - pi; %dtheta
    xdist = sqrt(x_des(1)*x_des(1) + x_des(2)*x_des(2));
    v_track(t) = kP*v*(xdist-tooClose)/tooClose; % match target speed v
    phi_track(t) = phi_track(t) + ang_err*kP; % phi(t+1) control collisions
    if (xdist < tooClose) % stop if too close
        v_track(t) = 0; 
        phi_track(t) = phi_track(t);
    end
    % Motion
    x_track(t,1) = x_track(t-1,1) + v_track(t)*cos(x_track(t-1,3))*dt + e(1);
    x_track(t,2) = x_track(t-1,2) + v_track(t)*sin(x_track(t-1,3))*dt + e(2);
    x_track(t,3) = x_track(t-1,3) + phi_track(t)*dt + e(3);
    
    %Mapping - c)
    %add targetbot to map - d)
    map_t = im2bw(insertShape(255*uint8(map), 'FilledCircle', ...
                             [x_target(t,2), x_target(t,1), 10], ...
                             'Color', 'black'));
    meas_phi = zeros(s_n,1); % angles to measure 
    for i=1:s_n % update map for each heading
        meas_phi(i) = (i/s_n - 0.5)*s_fov + x_track(t,3);
        meas_phi(i) = mod(meas_phi(i) + pi, 2*pi) - pi; % restrain to 360°, redundant
    end
    s = getranges(map,x_track(t-1,:), meas_phi, s_range); %scan map
    % Control modification for collisions
    closeto = find(s(s_n/8:7*s_n/8) == min(s(s_n/8:7*s_n/8)),1);
    if (s(closeto)< 10) %touching something
        phi_track(t+1) = -0.75; % turn more next iteration, better luck next time
    end
    
    % from scanreg2Dtry.m; Update probability maps
    for i = 1:length(meas_phi)
        % Get inverse measurement model (with scan registration estimate or
        % motion prediction)
        invmod = inversescannerbres(M,N,x_track(t,1),x_track(t,2),meas_phi(i),s(1,i),s_range);
        for j = 1:length(invmod(:,1));
            ix = invmod(j,1);
            iy = invmod(j,2);
            il = invmod(j,3);
            % Calculate updated log odds
            L(ix,iy) = L(ix,iy) +log(il./(1-il))-L0(ix,iy);
            s_map(ix,iy)=exp(L(ix,iy))./(1+exp(L(ix,iy)));
        end
    end
    
    figure(3); clf; hold on;
    colormap('gray');
    imagesc(1-s_map');    
    plot(x_track(1:t,1),x_track(1:t,2),'g');
    plot(x_target(1:t,1),x_target(1:t,2),'r');
    lt = legend('Tracker','Target');
    title('Complete Run of Tracking Map, 200 seconds');
    %drawbot(x_track(t,1),x_track(t,2),x_track(t,3),10,3);
    axis equal
    drawnow;
    
    % Plotting scene (slow, comment out if not needed)
%     figure(1); clf; hold on;
%     colormap('gray');
%     imagesc(1-map');
%     plot(tour(:,1),tour(:,2),'w')
%     drawbot(x_track(t,1),x_track(t,2),x_track(t,3),10,1); % Draw the tracking robot
%     %targetbot = drawbot(x_target(t,1),x_target(t,2),atan2(motion_dir(2), motion_dir(1)),10,1);
%     axis equal
%     drawnow;
% 
%     % Plotting map (slow, comment out if not needed)
%     figure(2); clf; hold on;
%     colormap('gray');
%     curmap = addbot2map(map, targetbot);
%     imagesc(1-curmap');
%     axis equal
%     drawnow;
end

