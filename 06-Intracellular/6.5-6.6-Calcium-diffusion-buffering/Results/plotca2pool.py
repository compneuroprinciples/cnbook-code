# Import and plot calcium concentrations for 1 pool vs 2 pool models
# (1) 1um diam, 0.1um shell
# (2) 1um diam, 0.01um shell
# (3) 4um diam, 0.1um shell
# BPG 4-11-06

from pylab import *

dt=0.025    # time step (msecs)

ca1 = load('cai_1p_1um.dat')
ca2 = load('cais_2p_1ump1um.dat')
ca3 = load('caic_2p_1ump1um.dat')
t=arange(0.,ca1.shape[0]*dt,dt)  # time: start, finish, time step (msecs)

subplot(231)
plot(t,ca2*1000,'k-')
hold(True)
plot(t,ca3*1000,'k--')
plot(t,ca1*1000,'k:')
xlabel('Time (msecs)')
ylabel('Concentration (uM)')
title('(a)')
legend(['submem', 'core', '1 pool'])

ca1 = load('cai_1p_1um.dat')
ca2 = load('cais_2p_1ump01um.dat')
ca3 = load('caic_2p_1ump01um.dat')
t=arange(0.,ca1.shape[0]*dt,dt)  # time: start, finish, time step (msecs)

subplot(232)
plot(t,ca2*1000,'k-')
hold(True)
plot(t,ca3*1000,'k--')
plot(t,ca1*1000,'k:')
xlabel('Time (msecs)')
#ylabel('Concentration (uM)')
title('(b)')
#legend(['submem', 'core', '1 pool'])

ca1 = load('cai_1p_4um.dat')
ca2 = load('cais_2p_4ump1um.dat')
ca3 = load('caic_2p_4ump1um.dat')
t=arange(0.,ca1.shape[0]*dt,dt)  # time: start, finish, time step (msecs)

subplot(233)
plot(t,ca2*1000,'k-')
hold(True)
plot(t,ca3*1000,'k--')
plot(t,ca1*1000,'k:')
xlabel('Time (msecs)')
#ylabel('Concentration (uM)')
title('(c)')
#legend(['submem', 'core', '1 pool'])

#savefig('casimp.png')
savefig('ca2pool.eps')
show()
