clc
% clear all

Q = ones(2);
theta_r_km1 = 0;
theta_l_km1 = 0;
theta_r_k = pi;
theta_l_k = pi;
R = 0.5;
L = 1;

z_k_gps = [0.9, 0.1, pi/6]';
R_gps = ones(3);

x_k = [1,0,pi/4]';
P_k = ones(3);

% [estX, estP] = vehicleCPU({0, 1, [{Q}, theta_r_km1, theta_l_km1, theta_r_k, theta_l_k, R, L], [{z_k_gps}, R_gps], [], [{x_k}, {P_k}]})
% a = send(1, Noise, Sensor, Vehicles, [0,0], 5, {[1,1,1]',ones(3)});

a = send(2, Noise, Sensor, Vehicles, [0,1], 5, {[1,1,1]',ones(3)});
[estX, estP] = vehicleCPU(a)
clear a