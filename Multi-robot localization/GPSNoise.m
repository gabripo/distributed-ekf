function Noisyq_m = GPSNoise(Vehicles, Noise)

% % Measurement is affected by Gaussian Noise 
% a = q(:,1) + (randn(1, length(q(:,1)))*Noise.GPS.R(1,1))' + Noise.GPS.mu(1) ;
% b = q(:,2) + (randn(1, length(q(:,2)))*Noise.GPS.R(2,2))' + Noise.GPS.mu(2) ;
% c = q(:,3) + (randn(1, length(q(:,3)))*Noise.GPS.R(3,3))' + Noise.GPS.mu(3) ;

Noisyq_m = zeros(length(Vehicles.x), 3*Vehicles.Num);
for i=1:3*Vehicles.Num
    Noisyq_m(:, i) = Vehicles.x(:, i) + (randn(1, length(Vehicles.x(:,i)))*Noise.GPS.R(i,i))' + Noise.GPS.mu(i) ;
end

% Noisyq_m = [a, b, c];