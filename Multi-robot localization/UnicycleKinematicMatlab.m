%% Function to simulate n vehicles by the unicycle model
function [q, t, u] = UnicycleKinematicMatlab(SimSets, Vehicles)

% Simulation set-up
Dt = SimSets.Ts;
t = 0:Dt:SimSets.T;

% Unicycle dynamic
[t, q] = ode45(@(t,y) UnicycleModel(t,y, Vehicles.Num), t, Vehicles.x0);

% Input sequence - SAME INPUT
[v, omega] = UnicycleInputs(t);
u = [v'; omega'];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Additional functions


function dy = UnicycleModel(t,y, num)

[v, omega] = UnicycleInputs(t);

% System kinematic
dy = zeros(1, 3*num)';
for i=1:num
    dy(i*3-2) = v*cos(y(i*3));
    dy(i*3-1) = v*sin(y(i*3));
    dy(i*3) = omega;
end


function [v, omega] = UnicycleInputs(t)

v = 1*ones(length(t),1);    % m/s
omega = 2*sin(2*pi*t/10).*cos(2*pi*t/2); % rad/s




