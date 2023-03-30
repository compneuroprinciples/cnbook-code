# Import and plot calcium concentrations for multishell models
# 2, 4 and 10 shells
# (1) 1um diam, 0.1um shell
# (2) 4um diam, 0.1um shell
# BPG 11-11-06

from pylab import *

dt=0.025    # time step (msecs)

ca2s = load('cais_2p_1ump1um.dat')
ca2c = load('caic_2p_1ump1um.dat')
ca4s = load('cais_4s_1ump1um.dat')
ca4c = load('caic_4s_1ump1um.dat')
ca10s = load('cais_10s_1ump1um.dat')
ca10c = load('caic_10s_1ump1um.dat')
t=arange(0.,ca2s.shape[0]*dt,dt)  # time: start, finish, time step (msecs)

subplot(221)
plot(t,ca2s*1000,'k-')
hold(True)
plot(t,ca4s*1000,'k--')
plot(t,ca10s*1000,'k:')
#xlabel('Time (msecs)')
ylabel('Concentration (uM)')
title('(a)')
legend(['2 shell', '4 shell', '10 shell'])

subplot(222)
plot(t,ca2c*1000,'k-')
hold(True)
plot(t,ca4c*1000,'k--')
plot(t,ca10c*1000,'k:')
#xlabel('Time (msecs)')
#ylabel('Concentration (uM)')
title('(b)')

ca2s = load('cais_2p_4ump1um.dat')
ca2c = load('caic_2p_4ump1um.dat')
ca4s = load('cais_4s_4ump1um.dat')
ca4c = load('caic_4s_4ump1um.dat')
ca10s = load('cais_10s_4ump1um.dat')
ca10c = load('caic_10s_4ump1um.dat')
t=arange(0.,ca2s.shape[0]*dt,dt)  # time: start, finish, time step (msecs)

subplot(223)
plot(t,ca2s*1000,'k-')
hold(True)
plot(t,ca4s*1000,'k--')
plot(t,ca10s*1000,'k:')
xlabel('Time (msecs)')
ylabel('Concentration (uM)')
title('(c)')
#legend(['2 shell', '4 shell', '10 shell'])

subplot(224)
plot(t,ca2c*1000,'k-')
hold(True)
plot(t,ca4c*1000,'k--')
plot(t,ca10c*1000,'k:')
xlabel('Time (msecs)')
#ylabel('Concentration (uM)')
title('(d)')

#savefig('camulti.png')
savefig('camulti.eps')
show()
