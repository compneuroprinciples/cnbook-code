# Plot results from three pool model of calcium concentration
# BPG 7-11-06

from pylab import *

dt=0.025    # time step (msecs)

# import and plot calcium concentrations

ca21 = load('cais_2p_1ump1um.dat')
ca22 = load('caic_2p_1ump1um.dat')
ca31 = load('cais_3c_1ump1um.dat')
ca32 = load('caic_3c_1ump1um.dat')
ca33 = load('cail_3c_1ump1um.dat')

#figure(figsize=(8,4))

t=arange(0.,ca21.shape[0]*dt,dt)  # time: start, finish, time step (msecs)
subplot(231)
#plot(t,ca1*1000,'k:')
hold(True)
plot(t,ca31*1000,'k-')
plot(t,ca21*1000,'k--')
xlabel('Time (msecs)')
ylabel('Concentration (uM)')
title('(a) submembrane')
legend(('3 pool', '2 pool'))

subplot(232)
hold(True)
plot(t,ca32*1000,'k-')
plot(t,ca22*1000,'k--')
xlabel('Time (msecs)')
#ylabel('Concentration (uM)')
title('(b) core')

subplot(233)
plot(t,ca33*1000,'k-')
xlabel('Time (msecs)')
#ylabel('Concentration (uM)')
title('(c) colocal')

#savefig('ca3compt.png')
savefig('ca3pool.eps')
show()
