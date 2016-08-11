function [map] = addbot2map(map,bot)
%% addbot2map 
% Takes in a polygonal robot and a unit size occupancy grid map
% and adds the robot to the map.

xmin = floor(min(bot(:,1)));
xmax = ceil(max(bot(:,1)));
ymin = floor(min(bot(:,2)));
ymax = ceil(max(bot(:,2)));

for i=xmin:xmax
    for j=ymin:ymax
        if (inpolygon(i,j,bot(:,1),bot(:,2)))
            map(i,j) = 0;
        end
    end
end
