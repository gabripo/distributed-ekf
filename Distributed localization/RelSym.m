% Simulate the relative position detection between q_i and q_j
function q_rel = RelSym(q_i, q_j)

% Increments computation
delta_x = q_j(:,1) - q_i(:,1);
delta_y = q_j(:,2) - q_i(:,2);
theta_i = q_i(:,3);
theta_j = q_j(:,3);

% Bearing angle
z_b_i = atan2( (-sin(theta_i).*delta_x + cos(theta_i).*delta_y) ,...
    (cos(theta_i).*delta_x + sin(theta_i).*delta_y) );
z_b_j = atan2( (-sin(theta_j).*delta_x + cos(theta_j).*delta_y) ,...
    (cos(theta_j).*delta_x + sin(theta_j).*delta_y) );

%  Relative distance
z_d = sqrt( delta_x.^2 + delta_y.^2 );

% Relative orientation
z_o = theta_j - theta_i;

q_rel = [z_b_i, z_d, z_o, z_b_j];