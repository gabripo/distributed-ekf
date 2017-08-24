function [q, t, u] = UnicycleKinematicMatlabPlus(SimSets, Vehicles)
%% Function to simulate n vehicles by the unicycle model, aumented version that allows to choose the inputs. SimSets.u is a struct containing v and omega to use as inputs

% Simulation set-up
Dt = SimSets.Ts;
t = 0:Dt:SimSets.T;

% Unicycle dynamic
[t, q] = ode45(@(t,y) UnicycleModel(t,y, Vehicles.Num, SimSets.u), t, Vehicles.x0);

% % Input sequence - u structure is: [v1,omega1,v2,omega2,...,vn,omegan]
u = zeros(length(t), 2*Vehicles.Num);
for i=1:length(t)
        [u(i,1:2:end), u(i,2:2:end)] = UnicycleInputs(t(i), Vehicles.Num, SimSets.u);
end
u = u';

function dy = UnicycleModel(t,y, num, u)

[v, omega] = UnicycleInputs(t, num, u);

% System kinematic
dy = zeros(1, 3*num)';
for i=1:num
    dy(i*3-2) = v(i)*cos(y(i*3));   % x_dot
    dy(i*3-1) = v(i)*sin(y(i*3));   % y_dot
    dy(i*3) = omega(i);             % theta_dot
end

function [v, omega] = UnicycleInputs(t, num, u)

for i=1:num
    v(i) = feval(u{1,i},t);
    omega(i) = feval(u{2,i},t);
end