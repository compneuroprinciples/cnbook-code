COMMENT
A synaptic current with dual exponential function conductance defined by
        i = g * (v - e)      i(nanoamps), g(micromhos);
        where
         g = 0 for t < onset and
         g = gmax*((tau1*tau2)/(tau1-tau2)) *
                             (exp(-(t-onset)/tau1)-exp(-(t-onset)/tau2))
         for t > onset (tau1 and tau2 are fast and slow time constants)
BPG 14-3-98
ENDCOMMENT
					       
INDEPENDENT {t FROM 0 TO 1 WITH 1 (ms)}

NEURON {
	POINT_PROCESS DESynapse
	RANGE onset, tau1, tau2, gmax, e, i, g
	NONSPECIFIC_CURRENT i
}
UNITS {
	(nA) = (nanoamp)
	(mV) = (millivolt)
	(umho) = (micromho)
}

PARAMETER {
	onset=0 (ms)
	tau1=.2 (ms)	<1e-3,1e6>
	tau2=2 (ms)	<1e-3,1e6>
	gmax=0 	(umho)	<0,1e9>
	e=0	(mV)
	v	(mV)
}

ASSIGNED { i (nA)  g (umho)}

BREAKPOINT {
	g = gmax*((tau1*tau2)/(tau1-tau2))*duale((t-onset)/tau1,(t-onset)/tau2)
	i = g*(v - e)
}

FUNCTION duale(x,y) {
	if (x < 0 || y < 0) {
		duale = 0
	}else{
		duale = exp(-x) - exp(-y)
	}
}
