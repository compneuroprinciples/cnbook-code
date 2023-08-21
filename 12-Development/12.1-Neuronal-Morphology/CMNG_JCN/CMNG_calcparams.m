function [calcp] = CMNG_calcparams(simp, modp)
% CMNG_calcparams - Calculate parameters from user-defined values
% Continuum Model for Neurite Outgrowth with Autoregulation
% Version 1.0 (BPG & DRM 5-2-05)

calcp.alpha = modp.a/(modp.rg*modp.c0)
calcp.beta = (modp.g*modp.D)/(modp.rg*modp.rg*modp.c0*modp.c0)
calcp.gamma = modp.sg/(modp.rg*modp.c0)
calcp.phi = (modp.e0*modp.D)/(modp.rg*modp.c0)
calcp.rho = modp.el/modp.e0
calcp.sigma = modp.zl/(modp.e0*modp.c0)
calcp.theta  = modp.er/modp.e0;
calcp.t0 = modp.D/(modp.rg*modp.rg*modp.c0*modp.c0);  % nondimensionalise time
calcp.delaystep = round(modp.rdt/(calcp.t0*simp.dt));  % AR delay time step
calcp.jmax = round(simp.tmax/(calcp.t0*simp.dt))      % final time step index
calcp.dy = 1/simp.N;                   % nondimensional spatial step
