function [RightEnc, LeftEnc] = EncoderNoise(Sensor, Noise, n)

for i=1:n
    % Measurement noise
    RightEnc(:,i) = Sensor.Enc.Right(:,i) + randn(1,length(Sensor.Enc.Right(:,i)))'*Noise.Enc.sigma + Noise.Enc.mu;
    LeftEnc(:,i)  = Sensor.Enc.Left(:,i) + randn(1,length(Sensor.Enc.Right(:,i)))'*Noise.Enc.sigma + Noise.Enc.mu;

    % Quantization effect
    RightEnc(:,i) = round(RightEnc(:,i)/Noise.Enc.Quanta)*Noise.Enc.Quanta;
    LeftEnc(:,i)  = round(LeftEnc(:,i)/Noise.Enc.Quanta)*Noise.Enc.Quanta;
end
