function [n,Pv,frD,psr,Pr] = syn_npD(Pv0,P1,tauf,nT,kn,kr,ns,D,tauD,spt)
% function [n,Pv,frD,psr,Pr] = syn_npD(Pv0,P1,tauf,nT,kn,kr,ns,D,tauD,spt) - synapse model
% Synapse with background and activity-dependent vesicle replenishement,
% release facilitation and desensitisation of the postsynaptic response.
% Replenishment is from a infinite sized reserve pool.
% Model mode is either "release-state" (nT>0) or "vesicle-state" (nT=0)
% Pv0 - base level use fraction per AP
% P1 - increment in Use per AP
% tauf - relaxation time constant of facilitation (msecs)
% nT - number of release sites (assumed unlimited if nT=0)
% kn - rate of replenishment (/msec)
% kr - rate of undocking (/msec)
% ns - average no. of new vesicles mobilised at each spike
% D - fraction of desensitised receptors
% tauD - time constant of recovery from desensitisation (msecs)
% spt - vector of spike times (msecs)
% Returns size of RRVP (n), prob. of vesicle release (Pv),
% fraction of desensitised receptors (frD),
% postsynaptic response (psr) and vesicle release (Pr=np)
% B. Graham, Computing Science & Maths, University of Stirling
% Contact: b.graham@cs.stir.ac.uk
% Last update: 1-7-2011

% Generate interspike intervals from spike times
spcnt = length(spt);	% number of spikes
isi = zeros(1,spcnt-1);
for i=2:spcnt
   isi(i-1) = spt(i) - spt(i-1);
end;

% Generate PRs (probability of release)

if (nT > 0)
    taur = 1/(kn+kr);  % release-site model
    n0 = kn*nT/(kn+kr);   % steady-state
else
    taur = 1/kr;       % vesicle-state model
    n0 =  kn/kr;
end;
ffac = 1-exp(-isi./tauf);	% facilitation
frec = 1-exp(-isi./taur);	% vesicle depletion
frecD = 1-exp(-isi./tauD);	% desensitisation

n = zeros(1,spcnt);	% avg. no. of available vesicles (RRVP)
n(1) = n0;
Pv = zeros(1,spcnt);
Pv(1) = Pv0;
frD = zeros(1,spcnt);	% fraction desensitised receptors

for i=2:spcnt
   Pvp = Pv(i-1)+P1*(1-Pv(i-1));
   Pv(i) = Pvp + ffac(i-1)*(Pv0-Pvp);
   np = (1-Pv(i-1))*n(i-1);
   n(i) = np + frec(i-1)*(n0-np) + ns;
   frDp = frD(i-1) + (D*Pv(i-1)*n(i-1)*(1-frD(i-1)));
   frD(i) = frDp - frecD(i-1)*frDp;
end;

Pr = n.*Pv;			% prob. release
psr = Pr.*(1-frD);	% prob. release x fraction not desensitised

