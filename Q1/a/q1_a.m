% Deterministic and Random Sampling for maze solving
close all; clc; clear all;

% Compute time check (Waslander Lenovo W530 = 0.39-0.46 s)
tic;
inv(rand(2000));
toc;

% Each row of the map structure draws a single line of the maze
% with coordinates [x1 x2 y1 y2];
% Top left corner of maze is [0.5 0.5], 
% Bottom right corner is [col+0.5
% row+0.5]
row = 6;
col = row;
map = maze(row,col);
start = [0.5, 1.0];
finish = [col+0.5, row];
 
%% Multi-query PRM, created in batch
tic;
xMin = [0.5 0.5];
xMax = [xMin(1)+col xMin(2)+row]; % State bounds
xR = xMax-xMin;
tol = 1;
% Get milestones
nS = row*col; % Min allowed is (M-1)*(N-1) to hit all turns
samples = [xR(1)*rand(nS,1)+xMin(1) xR(2)*rand(nS,1)+xMin(2)];
% throw out samples out of bounds
samples = samples(find(samples(:,1) > xMin(1)),:);
samples = samples(find(samples(:,1) < xMax(1)),:);
samples = samples(find(samples(:,2) > xMin(2)),:);
samples = samples(find(samples(:,2) < xMax(2)),:);
%Assume no line collisions, bot volume = 0
milestones = [start; finish; samples];
figure(1); hold on;
plot(samples(:,1),samples(:,2),'k.');
plot(milestones(:,1),milestones(:,2),'m.');
nM = length(milestones(:,1));
disp('Time to generate milestones');
toc;

% Attempt to add closest p edges
tic;
p = 20;
e = zeros(nM,nM);
D = zeros*ones(nM,nM);
checkCounts = 0;
for i = 1:nM
    % Find closest neighbours
    for j = 1:nM
        d(j) = norm(milestones(i,:)-milestones(j,:));
    end
    [d2,ind] = sort(d);
    % Check for edge collisions (no need to check if entire edge is
    % contained in obstacles as both endpoints are in free space)
    for j=1:p
        cur = ind(j);
        if (i<cur)
            checkLine = [milestones(i,1),milestones(cur,1), ...
                         milestones(i,2),milestones(cur,2)];
            checkCounts = checkCounts + 1;
            if (~MyCheckCollision(checkLine, map))
                e(i,cur) = 1;
                e(cur,i) = 1;
                plot([milestones(i,1) milestones(cur,1)],[milestones(i,2) milestones(cur,2)],'m');
            end
        end
    end
end
disp('Time to connect roadmap');
toc;

% Find shortest path
tic;
[sp, sd] = shortestpath(milestones, e, 1, 2);
for i=1:length(sp)-1
    plot(milestones(sp(i:i+1),1),milestones(sp(i:i+1),2), 'go-', 'LineWidth',3);
end
disp('Time to find shortest path');
toc;

% Info for part a)
disp(sprintf('Number of nodes is %d', length(samples(:))));
disp(sprintf('Collision checks performed is %d', checkCounts));
