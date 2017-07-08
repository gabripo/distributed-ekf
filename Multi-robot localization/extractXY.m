function xy = extractXY(xyz)
%% Function to extract the X-Y components from a cell array that contains the X-Y-Z components

temp = [xyz{1,1:3:end}; xyz{1,2:3:end}];
xy = num2cell([temp(1,:), temp(2,:)]);