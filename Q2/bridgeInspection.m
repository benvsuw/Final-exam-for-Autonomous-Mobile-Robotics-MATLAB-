% Bridge Inspection

bridge = [0 0; 0 10; 10 10; 10 0; 0 0; NaN NaN;
          100 0; 110 0; 110 10; 100 10; 100 0; NaN NaN;
          20 0; 23 0; 23 3; 20 3; 20 0; NaN NaN;
          20 7; 23 7; 23 10; 20 10; 20 7; NaN NaN;
          80 0; 83 0; 83 3; 80 3; 80 0; NaN NaN;
          80 7; 83 7; 83 10; 80 10; 80 7];
      

% Initial quadrotor position
s = 4;
x0 = [105; 20] + s*randn(2,1);
quad = x0';

%sim
T = 10; %sec
dt = 0.5; %timestep
for t=1:dt:10


% Path
path = [105 15; 5 15; 15 15; 15 5; 95 5; 95 -5; 105 -5; 5 -5; 50 -5; 50 20; 105 20];
% Fiducials
known_fiducials = [23 1.5; 23 8.5; 83 1.5; 83 8.5];
nF = 100;
unknown_fiducials = rand(nF,2);
unknown_fiducials(:,1) = 110*unknown_fiducials(:,1);
unknown_fiducials(:,2) = 30*unknown_fiducials(:,2)-5;
ind = inpolygon(unknown_fiducials(:,1), unknown_fiducials(:,2), bridge(:,1), bridge(:,2));
unknown_fiducials = unknown_fiducials(~ind,:);

fids = FidInQuad(quad, unknown_fiducials, bridge);
quad(:) = measModel(quad, fids);

% Plot initial configuration
figure(1); clf; hold on;
plot(bridge(:,1),bridge(:,2), 'LineWidth',2); %bridge obstacles
plot(x0(1),x0(2), 'go', 'MarkerSize',8,'LineWidth',2); %Start pos
plot(path(:,1), path(:,2), 'g'); %desired path
plot(known_fiducials(:,1),known_fiducials(:,2), 'rx', 'MarkerSize',4,'LineWidth',1);
plot(unknown_fiducials(:,1),unknown_fiducials(:,2), 'mx', 'MarkerSize',4,'LineWidth',1);
axis equal
% Plot my map
figure(2); clf; hold on;
plot(path(:,1), path(:,2), 'g'); %desired path
plot(quad(1), quad(2), 'ro');
plot(fids(:,1)+quad(1), fids(:,2)+quad(2), 'o');
axis equal
end

