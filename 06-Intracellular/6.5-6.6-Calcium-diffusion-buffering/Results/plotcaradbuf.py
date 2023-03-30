# Import and plot calcium concentrations for multishell models
# 4 shells with fixed and mobile buffering
# 4um diam, 0.1um shell
# BPG 5-2-07

from pylab import *

dt=0.025    # time step (msecs)

ca4s = load('cais_4s_4ump1um.dat')
ca4c = load('caic_4s_4ump1um.dat')
ca4sbfs = load('cais_4sD4_bfs50um.dat')
ca4cbfs = load('caic_4sD4_bfs50um.dat')
ca4sbff = load('cais_4sD4_bff5um.dat')
ca4cbff = load('caic_4sD4_bff5um.dat')
ca4sbmf = load('cais_4sD4_bmf5um.dat')
ca4cbmf = load('caic_4sD4_bmf5um.dat')
t=arange(0.,ca4s.shape[0]*dt,dt)  # time: start, finish, time step (msecs)

subplot(221)
plot(t,ca4s*1000,'k-')
hold(True)
plot(t,ca4sbfs*1000,'k--')
#xlabel('Time (msecs)')
ylabel('Concentration (uM)')
title('(a)')
legend(['no buff', 'slow'])

subplot(222)
plot(t,ca4c*1000,'k-')
hold(True)
plot(t,ca4cbfs*1000,'k--')
#xlabel('Time (msecs)')
#ylabel('Concentration (uM)')
title('(b)')

subplot(223)
plot(t,ca4s*1000,'k-')
hold(True)
plot(t,ca4sbff*1000,'k--')
plot(t,ca4sbmf*1000,'k:')
xlabel('Time (msecs)')
ylabel('Concentration (uM)')
title('(c)')
legend(['no buff', 'fast', 'mobile'])

subplot(224)
plot(t,ca4c*1000,'k-')
hold(True)
plot(t,ca4cbff*1000,'k--')
plot(t,ca4cbmf*1000,'k:')
xlabel('Time (msecs)')
#ylabel('Concentration (uM)')
title('(d)')

#savefig('camulti.png')
savefig('caradbuf.eps')
show()
