function [C, C0, CN, l, k] = CMNG_step(C, C0, CN, CTD0, l, simp, modp, calcp)
% CMNG_step: Function to integrate model for a single time step
% Continuum Model for Autoregulatory Neurite Outgrowth
% Version 2.0 (DRM & BPG 19-7-04) with Time Delay possible at x=0 bdy cn
% Version 1.0 (DRM & BPG 8-7-04)
% Version 2.1 (BPG & DRM 27-1-03)
%  - restrictions: length >= l0 and C >= 0
% Version 2.0 (BPG & DRM 24-1-03)

% Initialise
lk = l;
C0k = C0;
CNk = CN;
Ck = C;

% Integrate model for a single time step    
% (corrector steps)
for k=1:simp.kmax
    lkp = lk;
    C0kp = C0k;
    CNkp = CNk;
    Ckp = Ck;
    lk = l + simp.dt*(CNkp-calcp.gamma);   % new length
    lk = max(lk,modp.l0);  % limit length to l0
%    [A, b]=CMNG_system(C, C0, CN, l, lk, simp, modp, calcp);
    [A, b]=CMNG_system(C, C0, CN, CTD0, l, lk, simp, modp, calcp);
    Ck = (A\b)';   % solve using LU decomposition
    Ck = max(Ck, 0);    % limit conc. to positive values
%    [C0k, CNk] = CMNG_bc(Ck, C0, CN, l, simp, modp, calcp);   % boundary conds
    [C0k, CNk] = CMNG_bc(Ck, C0, CN, CTD0, l, simp, modp, calcp);   % boundary conds
    C0k = max(C0k, 0);    % limit conc. to positive values
    CNk = max(CNk, 0);    % limit conc. to positive values
    % check for convergence
    if (abs(lk-lkp)<simp.ml & abs(C0k-C0kp)<simp.mc & abs(CNk-CNkp)<simp.mc & max(abs(C0k-C0kp))<simp.mc)
        break;
    end
end

% Finalise
l = lk;
C0 = C0k;
CN = CNk;
C = Ck;
