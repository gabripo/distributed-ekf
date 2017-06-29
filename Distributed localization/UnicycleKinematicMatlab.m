function [q, t, u] = UnicycleKinematicMatlab(MdlInit, Vehicle)

% Simulation set-up
Dt = MdlInit.Ts;
t = 0:Dt:MdlInit.T;

% Unicycle dynamic
[t, q] = ode45(@(t,y) UnicycleModel(t,y), t, Vehicle.q0);

% Input sequence
[v, omega] = UnicycleInputs(t);
u = [v'; omega'];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Additional functions


function dy = UnicycleModel(t,y)

% Input parser
xu = y(1);
yu = y(2);
thetau = y(3);

[v, omega] = UnicycleInputs(t);

% System kinematic
xu_d = cos(thetau)*v;
yu_d = sin(thetau)*v;
thetau_d = omega;

% Output
dy = [xu_d; yu_d; thetau_d];


function [v, omega] = UnicycleInputs(t)

v = 1*ones(length(t),1);    % m/s
omega = 2*sin(2*pi*t/10).*cos(2*pi*t/2); % rad/s




