function [C, C0, CN] = CMNG_ic(simp, modp, calcp)
% CMNG_ic: Function to calculate initial conditions for concentration
% Implements linear initial conditions
% Continuum Model for Autoregulatory Neurite Outgrowth
% Version 1.0 (DRM & BPG 5-2-05)

denominator=calcp.theta*(1+calcp.rho*calcp.phi*modp.l0)+calcp.rho;
constant1=calcp.phi*modp.l0*(calcp.sigma*calcp.theta-calcp.rho);
constant2=1+calcp.rho*calcp.theta*modp.l0+calcp.sigma;

C0 = constant2/denominator;
for i=1:simp.N-1
    C(i) = (constant1*i*calcp.dy/denominator)+(constant2/denominator);
end
CN = (constant1*simp.N*calcp.dy/denominator)+(constant2/denominator);

clear constant1 constant2 denominator;