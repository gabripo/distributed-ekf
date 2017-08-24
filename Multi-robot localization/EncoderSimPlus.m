function [RightEnc, LeftEnc] = EncoderSimPlus(Vehicles)

for i=1:Vehicles.Num
    % Motors angular velocities
    OmegaR = (2*Vehicles.u(i,:) + Vehicles.L(i)*Vehicles.u(2*i,:))/(2*Vehicles.R(i));
    OmegaL = (2*Vehicles.u(i,:) - Vehicles.L(i)*Vehicles.u(2*i,:))/(2*Vehicles.R(i));

    % Angular increments for 'Delta t'
    RightEnc(:,i) = [0, OmegaR(1:end-1).*diff(Vehicles.t')];
    LeftEnc(:,i) =  [0, OmegaL(1:end-1).*diff(Vehicles.t')];

    % Encoder values
    RightEnc(:,i) = cumsum(RightEnc(:,i));
    LeftEnc(:,i)  = cumsum(LeftEnc(:,i));
end