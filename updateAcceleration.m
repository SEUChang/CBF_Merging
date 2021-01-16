function coe = updateAcceleration(v0, vf, t0, tf, p0)
L = 400;
Q = [p0, v0, L, vf];
% Decentralized optimal control algorithm
T = [1/6*t0^3 1/2*t0^2 t0 1;
    1/2*t0^2 t0 1 0;
    1/6*tf^3 1/2*tf^2 tf 1;
    1/2*tf^2 tf 1 0];
coeTrans = T^-1 *Q';
coe = coeTrans';
end
