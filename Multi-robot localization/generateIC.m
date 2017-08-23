function ic = generateIC(vehNum, rangeX, rangeY, rangeT)
%% Function to generate random initial conditions for the vehNum vehicles: rangeX is the range of the X coordinate, rangeY is the range of the Y coordinate, rangeT is the range of the orientation
% The probability density function is uniform

% Initialization
ic = zeros(1, 3*vehNum);

% Random X values generation
x = rangeX(1) + (rangeX(2)-rangeX(1))*rand(vehNum,1);

% Random Y values generation
y = rangeY(1) + (rangeY(2)-rangeY(1))*rand(vehNum,1);

% Random T values generation
theta = rangeT(1) + (rangeT(2)-rangeT(1))*rand(vehNum,1);

% Writing random values
for i=1:vehNum
    ic(i*3-2) = x(i);
    ic(i*3-1) = y(i);
    ic(i*3) = theta(i);
end

% WARNING: Having the same values of X-Y coordinates is very rare but this
% condition is not included!