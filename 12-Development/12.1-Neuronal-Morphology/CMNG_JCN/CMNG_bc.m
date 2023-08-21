function [C0k, CNk] = CMNG_bc(Ck, C0, CN, CTD0, l, simp, modp, calcp)
% CMNG_bc: Function to calculate boundary conditions for concentration
% Ck = concentration at current corrector step
% C0 = conc. at previous time step at proximal end
% CN = conc. at previous time step at distal end
% l = length at previous time step
% Continuum Model for Autoregulatory-Time Delay Neurite Outgrowth
% Version 2.0 (DRM & BPG 19-7-04)
% Version 1.0 (DRM & BPG 8-7-04)
% Version 2.0.1 (BPG & DRM 27-1-03)

%C0k = (2*calcp.dy*calcp.phi*l/3)+(4*Ck(1)/3)-(Ck(2)/3)-(2*calcp.dy*calcp.phi*calcp.theta*l*C0/3);
C0k = (2*calcp.dy*calcp.phi*l/3)+(4*Ck(1)/3)-(Ck(2)/3)...
    -(2*calcp.dy*calcp.phi*calcp.theta*l*CTD0/3);
CNk = -(2*calcp.dy*calcp.phi*l*((calcp.rho*CN)-calcp.sigma)/3)+(4*Ck(simp.N-1)/3)-(Ck(simp.N-2)/3);
