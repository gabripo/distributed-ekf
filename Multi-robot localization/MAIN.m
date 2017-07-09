clear all
clc
close all

% Always generating the same random numbers (useful to have the same noise
% in all the tests)
rng('default');
rng(1);
%% Simulation set-up

% Sampling time
SimSets.Ts = 0.01;

% Simulation length
SimSets.T = 400;

% Vehicles number
Vehicles.Num = 2;

% Vehicles initial conditions
Vehicles.x0 = zeros(1, 3*Vehicles.Num);
Vehicles.x0(4:6) = [1.5 1 pi/6];
% Vehicles.x0(7:9) = [1.5 -1 -pi/6];

% Vehicles lengthes and wheel radius assigment
Vehicles.L = zeros(1, Vehicles.Num);
Vehicles.R = zeros(1, Vehicles.Num);
for i=1:Vehicles.Num
   Vehicles.L(i) = 0.56;
   Vehicles.R(i) = 0.15;
end
clear i

%% Simulation

% Vehicles simulations
tic
[Vehicles.x, Vehicles.t, Vehicles.u] = UnicycleKinematicMatlab(SimSets, Vehicles);
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

% Relative positions noise 
Noise.Rel.mu = zeros(3*Vehicles.Num,1);
Noise.Rel.MaxBearErr = pi/6;    % [rad]
Noise.Rel.MaxDistErr = 1;       % [m]
Noise.Rel.MaxOriErr = pi/3;     % [rad]
% for i=1:Vehicles.Num
%     Noise.Rel.R(i*3-2:i*3, i*3-2:i*3) =  diag([(Noise.Rel.MaxBearErr/3)^2, (Noise.Rel.MaxDistErr/3)^2, (Noise.Rel.MaxOriErr/3)^2]);
% end
Noise.Rel.R = [];
for i=1:nchoosek(Vehicles.Num, 2)
    Noise.Rel.R = blkdiag(Noise.Rel.R, diag([(Noise.Rel.MaxBearErr/3)^2, (Noise.Rel.MaxBearErr/3)^2, (Noise.Rel.MaxDistErr/3)^2, (Noise.Rel.MaxOriErr/3)^2]));
end                
clear i          
%% Sensors simulation

% Encoders
[Sensor.Enc.Right, Sensor.Enc.Left] = EncoderSim(Vehicles);

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
for i=1:Vehicles.Num-1
    for j=i+1:Vehicles.Num
       Sensor.Rel.x_rel = [Sensor.Rel.x_rel, RelSym(A{i}, A{j})];
    end
end
clear i j A

[Sensor.Rel.Noisyx_rel] = RelSymNoise(Sensor.Rel.x_rel, Noise);

%% EKF Initialization

% Number of samples
EKF.NumS = length(Vehicles.t);

% Initial states
EKF.x_est = zeros(3*Vehicles.Num, 1);

% Initial covariance matrix of the states
EKF.P = 1e2*eye(3*Vehicles.Num);

% Covariance matrix of the encoders
EKF.Q = [];
for i=1:Vehicles.Num
    EKF.Q = blkdiag(EKF.Q, diag([Noise.Enc.sigma^2, Noise.Enc.sigma^2]));
end
clear i

% Covariance matrix of the measurements
EKF.R = blkdiag(Noise.GPS.R, Noise.Rel.R(4,4), Noise.Rel.R(1,1), Noise.Rel.R(2,2), Noise.Rel.R(3,3)); %Noise.Rel.R);

% Storing all the iterations
EKF.x_store = zeros(3*Vehicles.Num, EKF.NumS);
EKF.x_store(:,1) = EKF.x_est;

% Jacobian of GPS measurements
H_gps = [];
for n=1:Vehicles.Num
    H_ii = [];
   for j=1:n-1
      H_ii =  [H_ii, zeros(3)];
   end
   clear j
   H_ii = [H_ii, eye(3)];
   for j=1:Vehicles.Num-n
       H_ii =  [H_ii, zeros(3)];
   end
   clear j
   H_gps = [H_gps; H_ii];
end
clear n

%% JACOBIAN MATRICES OF RELATIVE MEASUREMENTS HARDCODING
% H = subs(H_sym, x_sym, x);
tic
[H_d_sym, H_o_sym, H_b_sym] = JacRel(Vehicles.x0);
matlabFunction(H_d_sym, 'File', 'H_d_mf');
matlabFunction(H_o_sym, 'File', 'H_o_mf');
matlabFunction(H_b_sym, 'File', 'H_b_mf');
CodeTime.JacComp = toc;

% Symbolic state vector
x_sym = symbolizer(EKF.x_est, 'x');

%% EKF 
tic
for i=2:EKF.NumS
    %% Prediction
    
    % Angle increments
    DeltaEnc = zeros(2*Vehicles.Num, 1);
    for k=1:Vehicles.Num
        DeltaEnc(k*2-1) = Sensor.Enc.NoisyRight(i) - Sensor.Enc.NoisyRight(i-1);
        DeltaEnc(k*2) = Sensor.Enc.NoisyLeft(i) - Sensor.Enc.NoisyLeft(i-1);
    end
    clear k
    
    % Jacobian matrix w.r.t the states
    Fx_k = [];
    for k=1:Vehicles.Num
        Fx_k = blkdiag(Fx_k, JacOdo(Vehicles, DeltaEnc, k, i));
    end
    clear k
    
    % Jacobian matrix w.r.t the inputs
    Fu_k = [];
    for k=1:Vehicles.Num
        Fu_k = blkdiag(Fu_k, JacInp(Vehicles, k, i));
    end
    clear k
    
    x_k1 = EKF.x_est + Fu_k*DeltaEnc;
    P_k1 = Fx_k*EKF.P*Fx_k'+Fu_k*EKF.Q*Fu_k';
    
%     EKF.x_est = x_k1;   % Test with only the model prediction


    %% Update
    % JACOBIAN MATRICES WITH RESPECT TO THE STATES
    H = [];
    Z = [];
    % GPS Measures
    H = [H; H_gps];

    Z = [Z; Sensor.GPS.Noisyq_m(i,:)'];
    
    % Jacobian matrices of relative measurements
    x_cell = num2cell(x_k1');
    x_cell_xy = extractXY(x_cell);

%     % 1 - Relative bearing angles
% %     H_b = double(subs(H_b_sym, x_sym, x_k1'));    % SLOW variant
    H_b = H_b_mf(x_cell_xy{1,:});
    H = [H; H_b];
    
    Z = [Z; Sensor.Rel.Noisyx_rel(i,4); Sensor.Rel.Noisyx_rel(i,1)];   
    
    % 2 - Relative distance
%     H_d = double(subs(H_d_sym, x_sym, x_k1'));    % SLOW variant
    H_d = H_d_mf(x_cell_xy{1,:});

    H = [H; H_d];
    
    Z = [Z; Sensor.Rel.Noisyx_rel(i,2)];
    
    % 3 - Relative orientation
    H_o = H_o_mf;
    H = [H; H_o];
    
    Z = [Z; Sensor.Rel.Noisyx_rel(i,3)];
    
    % TODO Fix EKF.R size for more than 2 vehicles
    % Kalman gain computation
    K = P_k1*H'*inv(H*P_k1*H' + EKF.R);
    
    % Update equations
    EKF.x_est = x_k1 + K*(Z - H*x_k1);
    EKF.P = (eye(3*Vehicles.Num) - K*H)*P_k1;

    %% Storing the result
    EKF.x_store(:,i) = EKF.x_est;
end
CodeTime.EKF = toc;

%% PLOTTING
tic
% figure(1)
% plot(Vehicles.x(:,1), Vehicles.x(:,2), '--b', EKF.x_store(1,:),...
%     EKF.x_store(2,:), 'b', Vehicles.x(:,4), Vehicles.x(:,5), '--r',...
%     EKF.x_store(4,:), EKF.x_store(5,:), 'r')
% legend('Vehicle 1 - Exact trajectory', 'Vehicle 1 - Estimated trajectory',...
%     'Vehicle 2 - Exact trajectory', 'Vehicle 2 - Estimated trajectory')
% axis equal
% grid on

figure(2)
plot(Vehicles.t, EKF.x_store(1,:) - Vehicles.x(:,1)', 'b', Vehicles.t,...
    EKF.x_store(2,:) - Vehicles.x(:,2)', '--b', Vehicles.t,...
    EKF.x_store(4,:) - Vehicles.x(:,4)', 'r', Vehicles.t,...
    EKF.x_store(5,:) - Vehicles.x(:,5)', '--r')
legend('Vehicle 1 - e_x', 'Vehicle 1 - e_y', 'Vehicle 2 - e_x',...
    'Vehicle 2 - e_y')
grid on
CodeTime.Plot = toc