function Noisyq_rel = RelSymNoise(q_rel, Noise)

a = q_rel(:,1) + (randn(1, length(q_rel(:,1)))*Noise.Rel.R(1,1))' + Noise.Rel.mu(1) ;
b = q_rel(:,2) + (randn(1, length(q_rel(:,2)))*Noise.Rel.R(2,2))' + Noise.Rel.mu(2) ;
c = q_rel(:,3) + (randn(1, length(q_rel(:,3)))*Noise.Rel.R(3,3))' + Noise.Rel.mu(3) ;
d = q_rel(:,4) + (randn(1, length(q_rel(:,4)))*Noise.Rel.R(4,4))' + Noise.Rel.mu(4) ;

Noisyq_rel = [a, b, c, d];