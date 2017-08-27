function packet = send(id, Noise, Sensor, Vehicles, conditions, t, currentEst)
%% Function to select the interesting data by the vehicle's id

% Initialization of the packet to return (it will be a cell array)
packet = {};

% Unpack conditions about the availability of measurements
boolRel = conditions(1);
boolGPS = conditions(2);

if boolRel
    % Relative measurements are available
    if boolGPS
        % GPS measurements are available
        % Case IV - GPS and Relative measurements
        
        % Encoders data to send
        Q = diag([Noise.Enc.sigma^2, Noise.Enc.sigma^2]);
        theta_r_km1 = Sensor.Enc.NoisyRight(t-1);
        theta_l_km1 = Sensor.Enc.NoisyLeft(t-1);
        theta_r_k = Sensor.Enc.NoisyRight(t);
        theta_l_k = Sensor.Enc.NoisyLeft(t);
        R = Vehicles.R(id);
        L = Vehicles.L(id);
        
        % Previous state estimation
        x_k = currentEst{1};
        P_k = currentEst{2};
        
        % GPS data to send
        Zk_gps = Sensor.GPS.Noisyq_m(t,3*id-2:3*id)';
        Rk_gps = Noise.GPS.R(id*3-2:id*3, id*3-2:id*3);
        
    else
        % GPS measurements are NOT available
        % Case III - No GPS and Relative measurements
        
        % Encoders data to send
        Q = diag([Noise.Enc.sigma^2, Noise.Enc.sigma^2]);
        theta_r_km1 = Sensor.Enc.NoisyRight(t-1);
        theta_l_km1 = Sensor.Enc.NoisyLeft(t-1);
        theta_r_k = Sensor.Enc.NoisyRight(t);
        theta_l_k = Sensor.Enc.NoisyLeft(t);
        R = Vehicles.R(id);
        L = Vehicles.L(id);
        
        % Previous state estimation
        x_k = currentEst{1};
        P_k = currentEst{2};
        
        % All the relative measurements
        Zk_rel = Sensor.Rel.finalx_rel;
        
        % Bearing angles w.r.t id
        Zk_b = Zk_rel{id}(t,:);
        
        % Relative distances w.r.t id
        Zk_d = Zk_rel{Vehicles.Num+id}(t,:);
        
        % Relative orientations w.r.t id
        Zk_o = Zk_rel{2*Vehicles.Num+id}(t,:);
        
        % Packet to send
        packet = {boolRel, boolGPS, [{Q}, theta_r_km1, theta_l_km1,...
            theta_r_k, theta_l_k, R, L], [], {Zk_b, Zk_d, Zk_o},...
            [{x_k}, {P_k}]};
    end
    
else
    % Relative measurements are NOT available
    if boolGPS
        % GPS measurements are available
        % Case II - GPS and No Relative measurements
        
        % Encoders data to send
        Q = diag([Noise.Enc.sigma^2, Noise.Enc.sigma^2]);
        theta_r_km1 = Sensor.Enc.NoisyRight(t-1);
        theta_l_km1 = Sensor.Enc.NoisyLeft(t-1);
        theta_r_k = Sensor.Enc.NoisyRight(t);
        theta_l_k = Sensor.Enc.NoisyLeft(t);
        R = Vehicles.R(id);
        L = Vehicles.L(id);
        
        % Previous state estimation
        x_k = currentEst{1};
        P_k = currentEst{2};
        
        % GPS data to send
        Zk_gps = Sensor.GPS.Noisyq_m(t,3*id-2:3*id)';
        Rk_gps = Noise.GPS.R(id*3-2:id*3, id*3-2:id*3);
        
        % Packet to send
        packet = {boolRel, boolGPS, [{Q}, theta_r_km1, theta_l_km1,...
            theta_r_k, theta_l_k, R, L], {Zk_gps, Rk_gps}, [],...
            [{x_k}, {P_k}]};
        
    else
        % GPS measurements are NOT available
        % Case I - No GPS and No Relative measurements
        
        % Encoders data to send
        Q = diag([Noise.Enc.sigma^2, Noise.Enc.sigma^2]);
        theta_r_km1 = Sensor.Enc.NoisyRight(t-1);
        theta_l_km1 = Sensor.Enc.NoisyLeft(t-1);
        theta_r_k = Sensor.Enc.NoisyRight(t);
        theta_l_k = Sensor.Enc.NoisyLeft(t);
        R = Vehicles.R(id);
        L = Vehicles.L(id);
        
        % Previous state estimation
        x_k = currentEst{1};
        P_k = currentEst{2};
        
        % Packet to send
        packet = {boolRel, boolGPS, [{Q}, theta_r_km1, theta_l_km1,...
            theta_r_k, theta_l_k, R, L], [], [], [{x_k}, {P_k}]};
    end
end