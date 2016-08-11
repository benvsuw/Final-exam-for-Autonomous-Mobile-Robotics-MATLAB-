function meas_r = getranges(map,X,meas_phi, rmax)
% Generate range measurements for a laser scanner based on a map, vehicle
% position and sensor parameters.
% Rough implementation of ray tracing algorithm.

% Initialization
[M,N] = size(map);
x = X(1);
y = X(2);
%th = X(3);
meas_r = rmax*ones(size(meas_phi'));

% For each measurement bearing
for i=1:length(meas_phi)
    % For each unit step along that bearing up to max range
   for r=1:rmax
       % Determine the coordinates of the cell
       xi = round(x+r*cos(meas_phi(i)));
       yi = round(y+r*sin(meas_phi(i)));
       % If not in the map, set measurement there and stop going further 
       if (xi<=1||xi>=M||yi<=1||yi>=N)
           meas_r(i) = sqrt((x-xi)^2+(y-yi)^2);
           break;
       % If in the map but hitting an obstacle, set measurement range and
       % stop going further
       elseif (map(xi,yi) == 0)
           meas_r(i) = sqrt((x-xi)^2+(y-yi)^2);
           break;
       end
   end
end
