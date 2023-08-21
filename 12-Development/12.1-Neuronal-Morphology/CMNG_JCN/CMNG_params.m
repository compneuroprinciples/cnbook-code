function [simp, modp, calcp] = CMNG_params
% CMNG_parameters Parameter values for Continuum Model 
%   for Autoregulatory Neurite Outgrowth
% simp - simulation parameters
% modp - user-defined (dimensional) parameters
% calcp - calculated (non-dimensional) parameters
% Version 2.0 (BPG & DRM 24-1-03)

% Parameters

% simulation
simp.dt = 0.1;                  % time step
simp.tmax = 1500;                  % simulation time
simp.datat = 10;                 % data collection time step
simp.N = 100;                     % number of spatial points
simp.kmax = 10000;               % maximum corrector steps
simp.mc = 0.0001;              % tolerance on C;
simp.ml = 0.0001;              % tolerance on l;

% user-defined
modp.c0 = 10;                     % concentration scale
modp.c1 = 10;                       % autoregulatory ideal concentration
modp.l0 = 0.01;                   % initial (min) length;
modp.D = 30000;                    % diffusion constant
modp.a = 100;                      % active transport rate
modp.g = 0.002;                    % decay rate
% normally rg=el & sg=zl
modp.rg = 10;                   % growth rate constant
modp.sg = 100;                   % growth rate set point (threshold)
modp.e0 = 2e-5;                  % soma flux-source rate
modp.el = 1e-4;                     % growth tip flux-sink rate
modp.rg = modp.el;                   % growth cone flux-sink rate
modp.zl = modp.sg;                   % growth cone flux-source rate

% calculated
calcp.alpha     = modp.a/(modp.rg*modp.c0)
calcp.beta      = (modp.g*modp.D)/(modp.rg*modp.rg*modp.c0*modp.c0)
calcp.gamma     = modp.sg/(modp.rg*modp.c0)
calcp.phi       = (modp.e0*modp.D)/(modp.rg*modp.c0)
calcp.rho       = modp.el/modp.e0
calcp.theta     = modp.c0/modp.c1
calcp.sigma     = (modp.el*modp.sg)/(modp.e0*modp.rg*modp.c0)
calcp.delaystep = 0                 % no delay of delaystep*dt in x=0 bdy cn

calcp.jmax      = round(simp.tmax/simp.dt)      % final time step index
calcp.dy        = 1/simp.N;                   % nondimensional spatial step
