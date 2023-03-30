# Import and plot calcium concentrations for 1 pool model3
# (1) accumulation of calcium influx
# (2) simple decay
# (3) instantaneous pump
# BPG 4-11-06

from pylab import *

dt=0.025    # time step (msecs)

ca1 = load('cai_1pcurr.dat')
ca2 = load('cai_1pdecay.dat')
ca3 = load('cai_1pJpump.dat')
t=arange(0.,ca1.shape[0]*dt,dt)  # time: start, finish, time step (msecs)

plot(t,ca1*1000,'k-')
hold(True)
plot(t,ca2*1000,'k--')
plot(t,ca3*1000,'k-.')
xlabel('Time (msecs)')
ylabel('Concentration (uM)')
#title('Simple decay')
legend(['accum', 'decay', 'pump'])

#savefig('casimp.png')
savefig('ca1pool.eps')
show()
