function [Ct, C0t, CNt, lt] = CMNG_run(simp, modp, calcp, tch, newp)
% CMNG_run: Function to run model for a specified number of time steps
% Continuum Model for Autoregulatory-time delay Neurite Outgrowth
% Version 1.0 (DRM & BPG 5-2-05)
%      - allows a single change of parameter values during run

% Data collection variables
if (simp.datat > 0)
    ndata = floor(calcp.jmax/simp.datat)+1;
    lt = zeros(ndata, 1);
    C0t = zeros(ndata, 1);
    CNt = zeros(ndata, 1);
    Ct = zeros(ndata, simp.N-1);
end
% Time step for parameter change
jch = round(tch/(calcp.t0*simp.dt))      % change time step index

% Initial conditions
[C, C0, CN] = CMNG_ic(simp, modp, calcp);
l = modp.l0;   % initial neurite length
CTD0=C0;       % initial "time delayed" value
CTD=0;         % intialise "time delay" array
if (modp.rdt > 0)   % time delay
    CTD = zeros(calcp.jmax+1, 1);   % expand "time delay" array
end

% Run simulation
for j=0:calcp.jmax
    % change of parameters to get e.g. neurite retraction
    if (j == jch)
        modp = newp;
        [calcp] = CMNG_calcparams(simp, modp);
    end
    % data collection
    if (simp.datat > 0)
        if (mod(j,simp.datat) == 0)   % data collection time
            time=j*simp.dt*calcp.t0   % current time
            i = (j/simp.datat)+1;
            lt(i)=l;
            Ct(i,:)=C;
            C0t(i)=C0;
            CNt(i)=CN;
        end
    end
    % integrate model for a single time step
    [C, C0, CN, l, k] = CMNG_step(C, C0, CN, CTD0, l, simp, modp, calcp);
    if (k == simp.kmax)
        k   % show k and quit if max k reached
        break;
    end
    % update time delay array, if necessary
    if (modp.rdt > 0)   % time delay
        CTD(j+1)=C0;
        if j>calcp.delaystep
            CTD0=CTD(j-calcp.delaystep);
        end
    else
        CTD0=C0;   % current value (no delay)
    end
end
