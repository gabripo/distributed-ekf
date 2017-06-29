clear all
close all
clc

%% Simulation set-up
% Sampling time
MdlInit.Ts = 0.05;

% Length of simulation
MdlInit.T = 10;

%% Vehicles set-up
% Initial conditions of vehicles
Vehicles.A.q0 = [0; 1; pi/4];
Vehicles.B.q0 = [1; 1; pi/2];

% Vehicles wheel radius
Vehicles.A.R = 0.15;
Vehicles.B.R = 0.15;

% Vehicles inter-axle
Vehicles.A.L = 0.56;
Vehicles.B.L = 0.56;

% Unycicle simulations
[Vehicles.A.q, Vehicles.A.t, Vehicles.A.u] = UnicycleKinematicMatlab(MdlInit, Vehicles.A);
[Vehicles.B.q, Vehicles.B.t, Vehicles.B.u] = UnicycleKinematicMatlab(MdlInit, Vehicles.B);