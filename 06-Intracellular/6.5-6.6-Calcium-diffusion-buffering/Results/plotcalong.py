# Import and plot calcium concentrations for radial and longitudinal diffusion model
# with four submembrane shells
# 1um and 4um dendrite, 10um long with ten 1um compartments
# BPG 5-2-07

from pylab import *

dt=0.025    # time step (msecs)

ca1 = load('cai_4ssl_seg1D1.dat')
ca2 = load('cai_4ssl_seg2D1.dat')
ca3 = load('cai_4ssl_seg3D1.dat')
ca4 = load('cai_4ssl_seg5D1.dat')
ca5 = load('cai_4ssl_seg10D1.dat')
t=arange(0.,ca1.shape[0]*dt,dt)  # time: start, finish, time step (msecs)

subplot(221)
plot(t[0:ca2.shape[0]],ca1*1000,'k-')
hold(True)
plot(t[0:ca2.shape[0]],ca2*1000,'k--')
plot(t[0:ca2.shape[0]],ca4*1000,'k-.')
plot(t[0:ca2.shape[0]],ca5*1000,'k:')
#xlabel('Time (msecs)')
ylabel('Concentration (uM)')
title('(a)')
legend(['0.5', '1.5', '4.5', '9.5'])


ca1 = load('cai_4scl_seg1D1.dat')
ca2 = load('cai_4scl_seg2D1.dat')
ca3 = load('cai_4scl_seg3D1.dat')
ca4 = load('cai_4scl_seg5D1.dat')
ca5 = load('cai_4scl_seg10D1.dat')
t=arange(0.,ca1.shape[0]*dt,dt)  # time: start, finish, time step (msecs)

subplot(222)
plot(t[0:ca2.shape[0]],ca1*1000,'k-')
hold(True)
plot(t[0:ca2.shape[0]],ca2*1000,'k--')
plot(t[0:ca2.shape[0]],ca4*1000,'k-.')
plot(t[0:ca2.shape[0]],ca5*1000,'k:')
#xlabel('Time (msecs)')
#ylabel('Concentration (uM)')
title('(b)')


ca1 = load('cai_4ssl_seg1D4.dat')
ca2 = load('cai_4ssl_seg2D4.dat')
ca3 = load('cai_4ssl_seg3D4.dat')
ca4 = load('cai_4ssl_seg5D4.dat')
ca5 = load('cai_4ssl_seg10D4.dat')
t=arange(0.,ca1.shape[0]*dt,dt)  # time: start, finish, time step (msecs)

subplot(223)
plot(t[0:ca2.shape[0]],ca1*1000,'k-')
hold(True)
plot(t[0:ca2.shape[0]],ca2*1000,'k--')
plot(t[0:ca2.shape[0]],ca4*1000,'k-.')
plot(t[0:ca2.shape[0]],ca5*1000,'k:')
xlabel('Time (msecs)')
ylabel('Concentration (uM)')
title('(c)')
legend(['0.5', '1.5', '4.5', '9.5'])


ca1 = load('cai_4scl_seg1D4.dat')
ca2 = load('cai_4scl_seg2D4.dat')
ca3 = load('cai_4scl_seg3D4.dat')
ca4 = load('cai_4scl_seg5D4.dat')
ca5 = load('cai_4scl_seg10D4.dat')
t=arange(0.,ca1.shape[0]*dt,dt)  # time: start, finish, time step (msecs)

subplot(224)
plot(t[0:ca2.shape[0]],ca1*1000,'k-')
hold(True)
plot(t[0:ca2.shape[0]],ca2*1000,'k--')
plot(t[0:ca2.shape[0]],ca4*1000,'k-.')
plot(t[0:ca2.shape[0]],ca5*1000,'k:')
xlabel('Time (msecs)')
#ylabel('Concentration (uM)')
title('(d)')


#savefig('calong.png')
savefig('calong.eps')
show()
