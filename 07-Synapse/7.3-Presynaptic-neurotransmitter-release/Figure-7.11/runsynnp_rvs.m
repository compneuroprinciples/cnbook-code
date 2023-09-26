% runsynnp-rvs.m
% Principles of Computational Modelling in Neuroscience
% Sterratt, Graham, Gillies, Willshaw
% Cambridge University Press, June 2011
% Fig. 7.11: Facilitation and depression in simple dynamic synapse.
% Run either the release-site or vesicle-state model.
% Stochastic versions of models.
% Simulation takes a little time (a minute or so) to run.
% B. Graham, Computing Science & Maths, University of Stirling
% Contact: b.graham@cs.stir.ac.uk
% Last update: 1-7-2011

% Parameters
slen=400;
freq=50; %(Hz)
Pv0=0.2;
P1=0.05;
tauf=100; % msecs
nT=0;   % (set to 0 for vesicle-state model)
kn=0.0002; % /msec (time const. 5secs)
kr=0.0002; % /msec (time const. 5secs)
ns=0.1;
D=1;
tauD=50;

line1='k-';
line2='bo';

%Generate spikes
isi = 1000/freq;	% interspike interval (msecs)
last = slen - rem(slen,isi);
spt = [isi:isi:last];	% spike times (including first recovery time)
spcnt = last/isi;	% no. of spikes (includes recovery spike)

% Run synapse model (single trials)
[n,Pv,frD,psr,Pr] = syn_npDs(Pv0,P1,tauf,nT,kn,kr,ns,D,tauD,spt);

% Run synapse model (multiple trials)
nrun=10000;  % number of trials
nav = zeros(1,spcnt);	% average no. of available vesicles (RRVP)
Prav = zeros(1,spcnt);	% average no. of available vesicles (RRVP)
psrav = zeros(1,spcnt);	% average no. of available vesicles (RRVP)
for i=1:nrun
    [n,Pv,frD,psr,Pr] = syn_npDs(Pv0,P1,tauf,nT,kn,kr,ns,D,tauD,spt);
    nav = nav+n;
    Prav = Prav+Pr;
    psrav = psrav+psr;
end;
nav = nav./nrun;    % average fraction n per RRVP size
Prav = Prav./nrun; % average fraction release per RRVP size
psrav = psrav./nrun; % average fraction response per RRVP size

tsize=9;
lsize=9;
nsize=9;
lwidth=0.8;

% Plot n, p and T=np
subplot(3,2,1);
mline=plot(spt,Pv,line1);
set(mline,'LineWidth',lwidth);
set(gca,'Box','off');
hold on;
title('(a) Single trial');
ylabel('p','FontSize',lsize,'FontName','Helvetica');
axis([0 400 0 0.4]);
subplot(3,2,3);
mline=plot(spt,n,line2);
set(mline,'LineWidth',lwidth);
set(gca,'Box','off');
hold on;
ylabel('n','FontSize',lsize,'FontName','Helvetica');
subplot(3,2,5);
mline=plot(spt,Pr,line2);
set(mline,'LineWidth',lwidth);
set(gca,'Box','off');
hold on;
xlabel('t (msecs)','FontSize',lsize,'FontName','Helvetica');
ylabel('T=np','FontSize',lsize,'FontName','Helvetica');

subplot(3,2,2);
mline=plot(spt,Pv,line1);
set(mline,'LineWidth',lwidth);
set(gca,'Box','off');
hold on;
title('(b) Average reponse');
axis([0 400 0 0.4]);
subplot(3,2,4);
mline=plot(spt,nav,line1);
set(mline,'LineWidth',lwidth);
set(gca,'Box','off');
hold on;
subplot(3,2,6);
mline=plot(spt,Prav,line1);
set(mline,'LineWidth',lwidth);
set(gca,'Box','off');
hold on;
xlabel('t (msecs)','FontSize',lsize,'FontName','Helvetica');

set(findobj('Type','line'),'LineWidth',lwidth);
set(findobj('Type','text'),'FontSize',nsize,'FontName','Helvetica');

