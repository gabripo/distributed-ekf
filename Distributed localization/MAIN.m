clear all
clc
close all

% Always generating the same random numbers (useful to have the same noise
% in all the tests)
rng('default');
rng(2);
%% Simulation set-up

% Sampling time
SimSets.Ts = 0.01;

% Simulation length
SimSets.T = 500;

% Vehicles number
Vehicles.Num = 4;

% Vehicles initial conditions
rangeX = [-2,2];
rangeY = [-2,2];
rangeT = [-pi/4,pi/4];
Vehicles.x0 = generateIC(Vehicles.Num, rangeX, rangeY, rangeT);
clear rangeX rangeY rangeT

% Vehicles lengthes and wheel radius assigment
Vehicles.L = zeros(1, Vehicles.Num);
Vehicles.R = zeros(1, Vehicles.Num);
for i=1:Vehicles.Num
   Vehicles.L(i) = 0.56;
   Vehicles.R(i) = 0.15;
end
clear i

% RELATIVE MEASUREMENTS ACTIVATION - 1 is "active relative measurements"
SimSets.actRel = 0;
boolRel = SimSets.actRel;
%% Simulation

% INPUTS
% Generator settings
funV = @(t) 1;
funOmega = @(t) 2*sin(2*pi*t/10).*cos(2*pi*t/2);

% Input generation function:
% Choose 'same' to have the same input for all the vehicles
% Choose'random' to have a random coefficient which multiplies v and omega
SimSets.u = inputGenerator(Vehicles.Num, funV, funOmega, 'same');

% Vehicles simulations
tic
% [Vehicles.x, Vehicles.t, Vehicles.u] = UnicycleKinematicMatlab(SimSets, Vehicles);
[Vehicles.x, Vehicles.t, Vehicles.u] = UnicycleKinematicMatlabPlus(SimSets, Vehicles);
CodeTime.VehicleSym = toc;
%% Noises - equal for all the vehicles

% Encoder quantization
Noise.Enc.Quanta = 2*pi/2600;

% Encoder noise
Noise.Enc.mu = 0;
Noise.Enc.sigma = 2*Noise.Enc.Quanta/3;

% GPS noise - only 2D coordinates (added orientation to test)
Noise.GPS.mu = zeros(3*Vehicles.Num,1);
Noise.GPS.MaxPosErr = 5;    % [m]
Noise.GPS.MaxOriErr = pi;
Noise.GPS.R = zeros(3*Vehicles.Num);
for i=1:Vehicles.Num
    Noise.GPS.R(i*3-2:i*3, i*3-2:i*3) =  diag([(Noise.GPS.MaxPosErr/3)^2, (Noise.GPS.MaxPosErr/3)^2, (Noise.GPS.MaxOriErr/3)^2]);
end
clear i

% Failure probability of the GPS
Noise.GPS.probFailure = 0;

% Relative positions noise 
Noise.Rel.mu = zeros(4*nchoosek(Vehicles.Num,2),1);
Noise.Rel.MaxBearErr = pi;    % [rad]
Noise.Rel.MaxDistErr = 5;       % [m]
Noise.Rel.MaxOriErr = pi;     % [rad]
Noise.Rel.R = [];
for i=1:nchoosek(Vehicles.Num, 2)
    Noise.Rel.R = blkdiag(Noise.Rel.R, diag([(Noise.Rel.MaxBearErr/3)^2, (Noise.Rel.MaxBearErr/3)^2, (Noise.Rel.MaxDistErr/3)^2, (Noise.Rel.MaxOriErr/3)^2]));
end                
clear i          
%% Sensors simulation

% Encoders
% [Sensor.Enc.Right, Sensor.Enc.Left] = EncoderSim(Vehicles);
[Sensor.Enc.Right, Sensor.Enc.Left] = EncoderSimPlus(Vehicles);

% Noisy encoders
[Sensor.Enc.NoisyRight, Sensor.Enc.NoisyLeft] = EncoderNoise(Sensor, Noise, Vehicles.Num);

% GPS
[Sensor.GPS.q_m] = GPSsym(Vehicles.x);

% Noisy GPS
[Sensor.GPS.Noisyq_m] = GPSNoise(Vehicles, Noise);

% Relative measurements
% [Sensor.Rel.x_rel] = RelSym(Vehicles.x(:,1:3), Vehicles.x(:,4:6));    %2D
Sensor.Rel.x_rel = [];
for i=1:Vehicles.Num
    A{i} = Vehicles.x(:,i*3-2:i*3);
end
clear i
% for i=1:Vehicles.Num-1
%     for j=i+1:Vehicles.Num
for i=Vehicles.Num:-1:2
    for j=1:i-1
       Sensor.Rel.x_rel = [Sensor.Rel.x_rel, RelSym(A{j}, A{i})];
    end
end
clear i j A

% Noisy relative measurements
[Sensor.Rel.Noisyx_rel] = RelSymNoise(Sensor.Rel.x_rel, Noise);

% REORDERED (noisy) relative measurements
Sensor.Rel.finalx_rel = extractXrel(Sensor.Rel.Noisyx_rel, Vehicles.Num);
%%  DIFFERENT FROM THE CENTRALIZED KALMAN FROM THIS POINT

%% EKF Initialization

% COSE DA FARE:
% 1) Creare script nel quale si differenziano i vari casi
% 2) Stabilire dal main code le variabili da inviare, creare programma di
% invio dati
% 3) Provare con solamente misure di encoders e GPS (quindi senza invio)


% Number of samples
EKF.NumS = length(Vehicles.t);

% Initial states
EKF.x_est = zeros(3*Vehicles.Num, 1);

% Initial covariance matrix of the states
EKF.P = 1e2*eye(3*Vehicles.Num);

%% EKF
tic
for i=2:EKF.NumS
    
%     x_est = EKF.x_est
%     P = EKF.P
    
%     parpool(Vehicles.Num)
%     parfor n=1:Vehicles.Num
%         
%         [x_est(n*3-2:n*3), P(n*3-2:n*3,n*3-2:n*3)] =...
%             vehicleCPU(send(n, Noise, Sensor, Vehicles, [0,1], i,...
%                 {x_est(n*3-2:n*3), P(n*3-2:n*3,n*3-2:n*3)}));
%     end
%     delete(gcp('nocreate'))

    if rand(1) > Noise.GPS.probFailure
        % Working GPS
        boolGPS = 1;
    else
        boolGPS = 0;
    end
    
    for n=1:Vehicles.Num
        
        [EKF.x_est(n*3-2:n*3), EKF.P(n*3-2:n*3,n*3-2:n*3)] =...
            vehicleCPU(send(n, Noise, Sensor, Vehicles,...
            [boolRel,boolGPS], i, {EKF.x_est(n*3-2:n*3),...
            EKF.P(n*3-2:n*3,n*3-2:n*3)}));
    end
    
    %% Storing the result
    EKF.x_store(:,i) = EKF.x_est;
end
CodeTime.EKF = toc;

%% PLOTTING
tic
figure(1)
% % 2D plot:
% plot(Vehicles.x(:,1), Vehicles.x(:,2), '--b', EKF.x_store(1,:),...
%     EKF.x_store(2,:), 'b', Vehicles.x(:,4), Vehicles.x(:,5), '--r',...
%     EKF.x_store(4,:), EKF.x_store(5,:), 'r')
% legend('Vehicle 1 - Exact trajectory', 'Vehicle 1 - Estimated trajectory',...
%     'Vehicle 2 - Exact trajectory', 'Vehicle 2 - Estimated trajectory')
comparePlot(Vehicles.x, EKF.x_store, 'Exact trajectory of', 'Estimated trajectory of')
axis equal
grid on
xlabel('X (m)')
ylabel('Y (m)')
saveas(gcf,'Trajectories','epsc')

figure(2)
% plot(Vehicles.t, EKF.x_store(1,:) - Vehicles.x(:,1)', 'b', Vehicles.t,...
%     EKF.x_store(2,:) - Vehicles.x(:,2)', '--b', Vehicles.t,...
%     EKF.x_store(4,:) - Vehicles.x(:,4)', 'r', Vehicles.t,...
%     EKF.x_store(5,:) - Vehicles.x(:,5)', '--r')
% legend('Vehicle 1 - e_x', 'Vehicle 1 - e_y', 'Vehicle 2 - e_x',...
%     'Vehicle 2 - e_y')
errorPlot(Vehicles.t, Vehicles.x, EKF.x_store)
grid on
xlabel('Time (s)')
ylabel('Error (m)')
saveas(gcf,'PositionErrors','epsc')
CodeTime.Plot = toc