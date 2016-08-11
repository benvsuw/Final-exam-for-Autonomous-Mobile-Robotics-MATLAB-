function inCollision = MyCheckCollision(line, map)
%CHECKCOLLISION Checks if a line interesects with a set of edges
%   Take in line [x1, x2, y1, y2] and map [xi1 xi2 yi1 yi2]
%   Returns boolean true if line intersects with any edge
if (size(line(1,:)) ~= size(map(1,:)))
   error('Line and map must be the same dimensions.\nLine = %d, map = %d', uint8(size(line(1,:))), uint8(size(map(1,:))));
end
inCollision = zeros(length(line(:,1)),1);
% Check for each edge
for k=1:size(line(:,1))
    x1=line(k,1);
    x2=line(k,2);
    y1=line(k,3);
    y2=line(k,4);
    for j=1:size(map(:,1))
        if (inCollision(k))
            break
        end
        xi1=map(j,1);
        xi2=map(j,2);
        yi1=map(j,3);
        yi2=map(j,4);
        
        %% Main Loop
        % if in rect
        if ((min(x1,x2) > max(xi1, xi2)) || ...
            (max(x1,x2) < min(xi1, xi2)) || ...
            (min(y1,y2) > max(yi1, yi2)) || ...
            (max(y1,y2) < min(yi1, yi2)))
            %no collision
            continue % next j
        end
        
        alpha = atan2(yi1-y1,xi1-x1);
        beta = atan2(yi2-y1,xi2-x1);
        dab = mod(beta - alpha + pi, 2*pi) - pi; % restrict angles
        phi = atan2(y2-y1,x2-x1);
        dap = mod(phi - alpha + pi, 2*pi) - pi; %Difference Alpha Phi
        if ((dab >= 0 && dap >= 0 && dab >=  dap) || ...
            (dab <= 0 && dap <= 0 && dab <= dap)) % dab/dap > +1 ?
            % Line pointing towards edge
            % Project (x1,y1) onto edge, find distance
            dist = abs((xi2-xi1)*(yi1-y1) - (xi1 - x1)*(yi2-yi1))/ ...
                   sqrt((xi2-xi1)^2 + (yi2-yi1)^2);
            n = [yi2-yi1, -(xi2-xi1)]; %normal to edge
            nang = atan2(n(2),n(1));
            nang = mod(nang-phi+pi,2*pi)-pi;
            magLine = sqrt((x2-x1)^2 + (y2-y1)^2);
            inCollision(k) = (abs(magLine*cos(nang)) >= dist);
        else
            continue
        end
    end
end