function [estX, estP] = vehicleCPU(id, data)
%% Function to simulate the processor of the id-th vehicle. It receives EKF which is a structure that contains P and Q of the vehicle states/inputs, cond which establishes the measurements data of the EKF

%% DATA UNPACKING
% Relative measurements boolean
boolRel = data{1};
% GPS measurements boolean
boolGPS = data{2};

% Encoders data
dataEnc = data{3};

% GPS data - dataGPS={} if GPS connection failed
dataGPS = data{4};

% Relative measurements data - dataRel={} if Relative measurements
% connection failed
dataRel = data{5};

% Old states estimation
dataOld = data{6};

%% EKF STEP
% The following code represents only one loop of an EKF execution (all the
% vehicles are synchronized with a master clock then each EKF loop lasts
% the same time for all the vehicles)

if boolRel
    % Relative measurements are available
    if boolGPS
        % GPS measurements are available
        % Case IV - GPS and Relative measurements
        
    else
        % GPS measurements are NOT available
        % Case III - No GPS and Relative measurements
        
        
    end
    
else
    % Relative measurements are NOT available
    if boolGPS
        % GPS measurements are available
        % Case II - GPS and No Relative measurements
        %% Prediction
        % Unpacking dataEnc
        [Q, oldencRight, oldencLeft, encRight, encLeft, R, L] = dataEnc{:};
        
        % Unpackind dataOld
        [X_k, P_k] = dataOld{:};
        theta_k = X_k(3);
        
        % Angle increments
        DeltaEnc = [ encRight - oldencRight, encLeft - oldencLeft ];
        
        % Jacobian matrix w.r.t the states
        Fx_k = JacOdo(R, theta_k, DeltaEnc);
        
        % Jacobian matrix w.r.t the inputs
        Fu_k = JacInp(theta_k, R, L);
        
        x_k1 = X_k + Fu_k*DeltaEnc';
        P_k1 = Fx_k*P_k*Fx_k'+Fu_k*Q*Fu_k';
        
        %% Update
        % JACOBIAN MATRICES WITH RESPECT TO THE STATES
        H = [];
        Z = [];
        R = [];
        
        % GPS Jacobian
        H_gps = eye(3);
        H = [H; H_gps];
        
        % Unpacking dataGPS
        [Z_gps, R_gps] = dataGPS{:};
        
        % Adding the GPS measurement
        Z = [Z; Z_gps];
        R = blkdiag(R, R_gps);
        
        % Kalman gain computation
        K = P_k1*H'*inv(H*P_k1*H' + R);
        
        % Update equations - H*x_k1 are the GPS measures
        estX = x_k1 + K*(Z - H*x_k1);
        estP = (eye(3) - K*H)*P_k1;
        
    else
        % GPS measurements are NOT available
        % Case I - No GPS and No Relative measurements
        %% Prediction
        % Unpacking dataEnc
        [Q, oldencRight, oldencLeft, encRight, encLeft, R, L] = dataEnc{:};
        
        % Unpackind dataOld
        [X_k, P_k] = dataOld{:};
        theta_k = X_k(3);
        
        % Angle increments
        DeltaEnc = [ encRight - oldencRight, encLeft - oldencLeft ];
        
        % Jacobian matrix w.r.t the states
        Fx_k = JacOdo(R, theta_k, DeltaEnc);
        
        % Jacobian matrix w.r.t the inputs
        Fu_k = JacInp(theta_k, R, L);
        
        estX = X_k + Fu_k*DeltaEnc';
        estP = Fx_k*P_k*Fx_k'+Fu_k*Q*Fu_k';
        
        %% Update
        
        % NO UPDATE DUE TO LACK OF MEASUREMENTS!
        end
end