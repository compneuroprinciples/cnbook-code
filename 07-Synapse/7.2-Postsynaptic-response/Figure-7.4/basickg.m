% basickg.m
% Principles of Computational Modelling in Neuroscience
% Sterratt, Graham, Gillies, Willshaw
% Cambridge University Press, June 2011
% Fig. 7.4: Basic 2-state kinetic conductance waveforms
% B. Graham, Computing Science & Maths, University of Stirling
% Contact: b.graham@cs.stir.ac.uk
% Last update: 1-7-2011

tmax=8;
dt=0.01;
t=0:dt:tmax;
tau=1;
tau2=3;
Tdur=1;
Tamp=1;
Ton=1;
alpha=1;
beta=1;
To=1/(alpha*Tamp+beta);
Tinf=alpha*Tamp/(alpha*Tamp+beta);

% 2-state kinetic scheme
Tr=zeros(size(t));
Tr(round(Ton/dt):round((Ton+Tdur)/dt))=1;
trise=t(round(Ton/dt):round((Ton+Tdur)/dt))-(Ton-dt);
tfall=t(round((Ton+Tdur)/dt)+1:length(t))-(Ton+Tdur);
gk=zeros(size(t));
gk(round(Ton/dt):round((Ton+Tdur)/dt))=Tinf.*(1-exp(-trise/To));
gp=gk(round((Ton+Tdur)/dt));
gk(round((Ton+Tdur)/dt)+1:length(t))=gp.*exp(-beta*tfall);
gk=gk./max(gk);

% Alpha function
twave=t(round(Ton/dt):length(t))-(Ton-dt);
gal=zeros(size(t));
galp=tau/exp(1);
gal(round(Ton/dt):length(t))=twave.*exp(-twave/tau)/galp;

tsize=9;
lsize=9;
nsize=9;

subplot(1,2,1);
plot(t,Tr,'k-');
title('(a)','FontSize',tsize,'FontName','Helvetica');
xlabel('t (msecs)','FontSize',lsize,'FontName','Helvetica');
ylabel('Concentration','FontSize',lsize,'FontName','Helvetica');
axis([0 tmax 0 1.02]);
set(gca,'Box','off');

subplot(1,2,2);
plot(t,gk,'k-');
hold on;
plot(t,gal,'k:');
title('(b)','FontSize',tsize,'FontName','Helvetica');
xlabel('t (msecs)','FontSize',lsize,'FontName','Helvetica');
ylabel('Conductance','FontSize',lsize,'FontName','Helvetica');
axis([0 tmax 0 1.02]);
set(gca,'Box','off');

set(findobj('Type','line'),'LineWidth',0.8);
set(findobj('Type','text'),'FontSize',nsize,'FontName','Helvetica');


