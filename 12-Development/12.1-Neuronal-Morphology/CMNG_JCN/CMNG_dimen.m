function [t, C, C0, CN, l] = CMNG_dimen(simp, modp, C, C0, CN, l)
% CMNG_dimen - Dimensionalise results
% Continuum Model for Neurite Outgrowth with Autoregulation
% Version 1.0 (BPG & DRM 5-2-05)

l = l*(modp.D/(modp.rg*modp.c0)); % real length
C = C*modp.c0;  % real concentration
C0 = C0*modp.c0;  % real concentration
CN = CN*modp.c0;  % real concentration

% dimensional time vector
i=0:length(l)-1;
t=i*simp.datat*simp.dt*modp.D/(modp.rg*modp.rg*modp.c0*modp.c0);
