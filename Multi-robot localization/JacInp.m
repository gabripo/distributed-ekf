%% Computes the Jacobian Matrix with respect to the inputs
function Fu_k = JacInp(Vehicles, i, k)

% Initializing the matrix to return at the end
Fu_k = zeros(3,2);

% Jacobian matrix
Fu_k = [cos(Vehicles.x(k, i*3))*Vehicles.R(i)/2, cos(Vehicles.x(k, i*3))*Vehicles.R(i)/2;
        sin(Vehicles.x(k, i*3))*Vehicles.R(i)/2, sin(Vehicles.x(k, i*3))*Vehicles.R(i)/2;
        Vehicles.R(i)/Vehicles.L(i), -Vehicles.R(i)/Vehicles.L(i)];
