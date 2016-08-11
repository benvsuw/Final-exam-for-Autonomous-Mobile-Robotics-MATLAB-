function fid = FidInQuad(quad, points, edges)
% takes in quad (x, y) and points (..,(xi,yi),..) and edges 
% of obstacles (..,(xi1 xi2 yi1 yi2),..)
% returns position relative to quad if in view
    keep = zeros(length(size(points(:,1))),1);
    fid = zeros(length(size(points(:,1))),2);
    %convert edges into proper format
    e = size(edges(:,1));
    lin = zeros(length(edges(:,1)),4);
    for k=1:length(edges(:,1))-1
        if (~mod(k,6) || ~mod(k+1,6)) 
            lin(k,:) = [NaN NaN NaN NaN];
            continue
        end
        lin(k,:) = [edges(k,1), edges(k+1,1), edges(k,2), edges(k+1,2)];
    end
    lin=lin(find(~isnan(lin(:,1))),:);
    for k=1:size(points(:,1))
        fid(k,1) = points(k,1) - quad(1);
        fid(k,2) = points(k,2) - quad(2);
        if ((abs(fid(k,1)) > 10) || (abs(fid(k,2)) > 7.5)) %within 20m x 15m 
            keep(k,1) = 0;
        else
            vec = [quad(1), points(k,1), quad(2), points(k,2)]; %reorder
            keep(k,1) = ~round(MyCheckCollision(vec, lin));
        end
    end
    fid = fid(find(keep(:)),:); %trim
end