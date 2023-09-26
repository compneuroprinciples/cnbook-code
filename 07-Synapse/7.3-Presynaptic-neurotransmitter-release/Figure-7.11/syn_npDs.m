function [n,Pv,frD,psr,Pr] = syn_npDs(Pv0,P1,tauf,nT,kn,kr,ns,D,tauD,spt)
% function [n,Pv,frD,psr,Pr] = syn_npDs(Pv0,P1,tauf,nT,kn,kr,ns,D,tauD,spt) - stochastic synapse model
% Synapse with background and activity-dependent vesicle replenishement,
% release facilitation and desensitisation of the postsynaptic response.
% Replenishment is from a infinite sized reserve pool.
% Model mode is either "release-state" (nT>0) or "vesicle-state" (nT=0)
% Stochastic vesicle recycling and release.
% Pv0 - base level use fraction per AP
% P1 - increment in Use per AP
% tauf - relaxation time constant of facilitation (msecs) 
% nT - number of release sites (assumed unlimited if nT=0)
% kn - rate of replenishment (/msec)
% kr - rate of undocking (/msec)
% ns - average no. of new vesicles mobilised at each spike is ns*n0
% D - fraction of desensitised receptors on release of whole RRVP
% tauD - time constant of recovery from desensitisation (msecs)
% spt - vector of spike times (msecs)
% Returns size of RRVP (n), prob. of vesicle release (pv),
% fraction of desensitised receptors (frD),
% postsynaptic response (psr) and vesicle release (Pr)
% B. Graham, Computing Science & Maths, University of Stirling
% Contact: b.graham@cs.stir.ac.uk
% Last update: 1-7-2011

%Generate interspike intervals
spcnt = length(spt);	% number of spikes
isi = zeros(1,spcnt-1);
for i=2:spcnt
   isi(i-1) = spt(i) - spt(i-1);
end;

% Generate PRs (probability of release)

dt = 100; % (msecs) time bin for vesicle recovery
nbin=100000/dt;   % number of bins in 100sec
% initialise RRVP from background activity over last 1 sec
n0 = 0;
for i=1:nbin
  n0 = n0 - sum(rand(1,n0)<=(kr*dt));   % spontaneous undocking
  if (nT > 0)
    n0 = n0 + (rand<=(kn*(nT-n0)*dt));   % release-site model
  else
    n0 = n0 + (rand<=(kn*dt));   % vesicle-state model
  end;
end;

ffac = 1-exp(-isi./tauf);	% facilitation
frecD = 1-exp(-isi./tauD);	% desensitisation

n = zeros(1,spcnt);	% exact no. of available vesicles (RRVP)
n(1) = n0;
Pv = zeros(1,spcnt);
Pv(1) = Pv0;
nrel = zeros(1,spcnt);	% number of released vesicles
nrel(1)=sum(rand(1,n(1))<=Pv(1));
frD = zeros(1,spcnt);	% fraction desensitised receptors

dt = 1; % (msecs) time bin for vesicle recovery
for i=2:spcnt
   Pvp = Pv(i-1)+P1*(1-Pv(i-1));
   Pv(i) = Pvp + ffac(i-1)*(Pv0-Pvp);
   if (n(i-1) > 0)   % probabilistic release of each vesicle
       n(i) = n(i-1)-nrel(i-1);
   end;
   % sum recovery for each time bin during next ISI
   nbin = round(isi(i-1)/dt);
   n(i) = n(i) - sum(rand(1,n(i))<=(kr*isi(i-1)));   % spontaneous undocking over ISI
   % background and activity-dependent replenishment over time bins
   if (nT > 0)
       nrep = sum(rand(1,nbin)<=(kn*(nT-n(i))*dt));   % release-site model
       nact = sum(rand(1,nbin)<=(ns*(nT-n(i))/nbin));   % average of ns over ISI
   else
       nrep = sum(rand(1,nbin)<=(kn*dt));   % vesicle-state model
       nact = sum(rand(1,nbin)<=(ns/nbin));   % average of ns over ISI
   end;
   n(i) = n(i)+nrep+nact;
   nrel(i)=sum(rand(1,n(i))<=Pv(i));
   frDp = frD(i-1) + (D*nrel(i-1)*(1-frD(i-1)));
   frD(i) = frDp - frecD(i-1)*frDp;
end;

Pr = nrel;			% number released
psr = Pr.*(1-frD);	% release x fraction not desensitised

