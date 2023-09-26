% pfacilk.m
% Principles of Computational Modelling in Neuroscience
% Sterratt, Graham, Gillies, Willshaw
% Cambridge University Press, June 2011
% Fig. 7.9: Kinetic gating, facilitating model of release probability
% B. Graham, Computing Science & Maths, University of Stirling
% Contact: b.graham@cs.stir.ac.uk
% Last update: 1-7-2011

tmax=500;
tm1=300;
t1s=198;
t1f=204;
isi=20;  % msecs (50Hz)
dt=0.1;
t=0:dt:tmax;
sp=zeros(1,length(t));
for i=1:floor(tmax/isi)
    sp(round(i*isi/dt))=1;  % spike times
end

% Kinetic 2-gate model, derived from Bertram et al, 1996
Caamp=1;   % amplitude of calcium transient (mM)
Cadur=1;    % duration (msecs)
Ca=0;
k1f=200; % forward rate (/mM-ms)
k1b=3;  % backward rate (/ms)
k2f=0.25; % forward rate (/mM-ms)
k2b=0.01;  % backward rate (/ms)
o1=zeros(1,length(t));
o2=zeros(1,length(t));
p3=zeros(1,length(t));
%p3(1)=p0;
tau1=1/(k1f*Caamp+k1b);
o1inf=k1f*Caamp*tau1;
tau2=1/(k2f*Caamp+k2b);
o2inf=k2f*Caamp*tau2;
for i=2:length(t)
  if (sp(i) == 1)   % spike, so turn on calcium transient
      Ca = Caamp;
      ton = i;
  end
  if (Ca == Caamp && i >= ton+(Cadur/dt))
      Ca = 0;   % transient finished
  end
  if (Ca > 0)
      o1(i) = o1inf+(o1(i-1)-o1inf)*exp(-dt/tau1);
      o2(i) = o2inf+(o2(i-1)-o2inf)*exp(-dt/tau2);
  else
      o1(i) = o1(i-1)*exp(-k1b*dt);
      o2(i) = o2(i-1)*exp(-k2b*dt);
  end;
  p3(i) = o1(i)*o2(i);
end;

tsize=9;
lsize=9;
nsize=9;
lwidth=0.8;

subplot(3,2,2);
plot(t,p3,'k-');
axis([t1s t1f 0 1.02]);
set(gca,'Box','off');

subplot(3,2,4);
plot(t,o1,'k-');
axis([t1s t1f 0 1.02]);
set(gca,'Box','off');

subplot(3,2,6);
plot(t,o2,'k-');
xlabel('t (msecs)','FontSize',lsize,'FontName','Helvetica');
axis([t1s t1f 0 1.02]);
set(gca,'Box','off');

subplot(3,2,1);
plot(t,p3,'k-');
ylabel('Probability','FontSize',lsize,'FontName','Helvetica');
axis([0 tm1 0 1.02]);
set(gca,'Box','off');

subplot(3,2,3);
plot(t,o1,'k-');
ylabel('Gate 1 Open','FontSize',lsize,'FontName','Helvetica');
axis([0 tm1 0 1.02]);
set(gca,'Box','off');

subplot(3,2,5);
plot(t,o2,'k-');
xlabel('t (msecs)','FontSize',lsize,'FontName','Helvetica');
ylabel('Gate 2 Open','FontSize',lsize,'FontName','Helvetica');
axis([0 tm1 0 1.02]);
set(gca,'Box','off');

set(findobj('Type','line'),'LineWidth',lwidth);
set(findobj('Type','text'),'FontSize',nsize,'FontName','Helvetica');

