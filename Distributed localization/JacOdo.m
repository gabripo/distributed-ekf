%% Computes the Jacobian matrix of the dinamics with respect to the states
function F_xi = JacOdo(R, theta_k, DeltaEnc)

% Initializing the matrix to return at the end
F_xi = zeros(3);

% Jacobian Matrix
F_xi =  [1 0 -sin(theta_k)*R/2*(DeltaEnc(1) + DeltaEnc(2));
         0 1 cos(theta_k)*R/2*(DeltaEnc(1) + DeltaEnc(2));
         0 0 1];
