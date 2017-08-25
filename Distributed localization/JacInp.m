%% Computes the Jacobian Matrix with respect to the inputs
function Fu_k = JacInp(theta_k, R, L)

% Initializing the matrix to return at the end
Fu_k = zeros(3,2);

% Jacobian matrix
Fu_k = [cos(theta_k)*R/2, cos(theta_k)*R/2;
        sin(theta_k)*R/2, sin(theta_k)*R/2;
        R/L, -R/L];
