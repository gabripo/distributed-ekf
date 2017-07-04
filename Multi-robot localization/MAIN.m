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
[Vehicles.x, Vehicles.t, Vehicles.u] = UnicycleKinematicMatlab(SimSets, Vehicles);

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

% Relative positions noise (for a couple of robots)
Noise.Rel.mu = zeros(3*Vehicles.Num,1);
Noise.Rel.MaxBearErr = pi/6;    % [rad]
Noise.Rel.MaxDistErr = 1;       % [m]
Noise.Rel.MaxOriErr = pi/3;     % [rad]
for i=1:Vehicles.Num
    Noise.Rel.R(i*3-2:i*3, i*3-2:i*3) =  diag([(Noise.Rel.MaxBearErr/3)^2, (Noise.Rel.MaxDistErr/3)^2, (Noise.Rel.MaxOriErr/3)^2]);
end
                    
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
[Sensor.Rel.x_rel] = RelSym(Vehicles.x(:,1:3), Vehicles.x(:,4:6));

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
EKF.R = blkdiag(Noise.GPS.R, Noise.Rel.R); % diag([Noise.Rel.R(1,1), Noise.Rel.R(3,3), Noise.Rel.R(4,4), Noise.Rel.R(6,6)])); %Noise.Rel.R);

% Storing all the iterations
EKF.x_store = zeros(3*Vehicles.Num, EKF.NumS);
EKF.x_store(:,1) = EKF.x_est;



%% EKF 

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
%     H_ii = [eye(3), zeros(3)];
%     H_jj = [zeros(3), eye(3)];
%     H = [H; H_ii; H_jj];
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
       H = [H; H_ii];
    end
    clear n

    Z = [Z; Sensor.GPS.Noisyq_m(i,:)'];
    
    % Relative measures definition
%     DX = x_k1(4) - x_k1(1);
%     DY = x_k1(5) - x_k1(2);
%     DT = x_k1(6) - x_k1(3);
    % Number of combinations without repetition
    combNum = nchoosek(Vehicles.Num, 2);
    DX = []; DY = []; DT = [];
    for n=1:combNum
       DX = [DX, x_k1(n+1:3:end) - x_k1(1:3:end-n)];
       DY = [DY, x_k1(n+1+1:3:end) - x_k1(1:3:end-n-1)];
       DT = [DT, x_k1(n+1+2:3:end) - x_k1(1:3:end-n-1)];
    end
    % TODO Sistemare da qui
    % 1 - relative distance
    z_d = sqrt(DX^2+DY^2);
    
    if z_d==0
        z_d = 1e-2;
    end
    
    H_d = [-DX/z_d, -DY/z_d, 0, DX/z_d, DY/z_d, 0;  % H_ij1, H_ij2
           DX/z_d, DY/z_d, 0, -DX/z_d, -DY/z_d, 0]; % H_ji2, H_ji1
    H = [H; H_d];
    
    Z = [Z; Sensor.Rel.Noisyx_rel(i,2); -Sensor.Rel.Noisyx_rel(i,2)];
    
    % 2 - Bearing Angle measurement
    z_b_i = atan2(-sin(x_k1(3))*DX + cos(x_k1(3))*DY, cos(x_k1(3))*DX + sin(x_k1(3))*DY);
    z_b_j = atan2(-sin(x_k1(6))*DX + cos(x_k1(6))*DY, cos(x_k1(6))*DX + sin(x_k1(6))*DY);    
    
    H_b = [DY/z_d^2, -DX/z_d^2, -1, -DY/z_d^2, DX/z_d^2, 0;
           -DY/z_d^2, DX/z_d^2, 0, DY/z_d^2, -DX/z_d^2, -1];
    H = [H; H_b];
    
    Z = [Z; Sensor.Rel.Noisyx_rel(i,1); Sensor.Rel.Noisyx_rel(i,4)];
%     Z = [Z; z_b_i; z_b_j];  % no noise measurement (to test)
    
    % 3 - Relative orientation
%     z_o = DT;
    
    H_o = [0, 0, -1, 0, 0, 1;
           0, 0, 1, 0, 0, -1];
    H = [H; H_o];
    
    Z = [Z; Sensor.Rel.Noisyx_rel(i,3); Sensor.Rel.Noisyx_rel(i,3)];
    
    % Kalman gain computation
    K = P_k1*H'*inv(H*P_k1*H' + EKF.R);
    
    % Update equations
    EKF.x_est = x_k1 + K*(Z - H*x_k1);
    EKF.P = (eye(3*Vehicles.Num) - K*H)*P_k1;

    %% Storing the result
    EKF.x_store(:,i) = EKF.x_est;
end


%% PLOTTING
figure(1)
plot(Vehicles.x(:,1), Vehicles.x(:,2), '--b', EKF.x_store(1,:),...
    EKF.x_store(2,:), 'b', Vehicles.x(:,4), Vehicles.x(:,5), '--r',...
    EKF.x_store(4,:), EKF.x_store(5,:), 'r')
legend('Vehicle 1 - Exact trajectory', 'Vehicle 1 - Estimated trajectory',...
    'Vehicle 2 - Exact trajectory', 'Vehicle 2 - Estimated trajectory')
axis equal
grid on

figure(2)
plot(Vehicles.t, EKF.x_store(1,:) - Vehicles.x(:,1)', 'b', Vehicles.t,...
    EKF.x_store(2,:) - Vehicles.x(:,2)', '--b', Vehicles.t,...
    EKF.x_store(4,:) - Vehicles.x(:,4)', 'r', Vehicles.t,...
    EKF.x_store(5,:) - Vehicles.x(:,5)', '--r')
legend('Vehicle 1 - e_x', 'Vehicle 1 - e_y', 'Vehicle 2 - e_x',...
    'Vehicle 2 - e_y')
grid on