%% Computes the Jacobian matrix of the dinamics with respect to the states
function F_xi = JacOdo(Vehicles, DeltaEnc, i, k)

% Initializing the matrix to return at the end
F_xi = zeros(3);

% Jacobian Matrix
F_xi =  [1 0 -sin(Vehicles.x(k, i*3))*Vehicles.R(i)/2*(DeltaEnc(i*2-1) + DeltaEnc(i*2));
         0 1 cos(Vehicles.x(k, i*3))*Vehicles.R(i)/2*(DeltaEnc(i*2-1) + DeltaEnc(i*2));
         0 0 1];
