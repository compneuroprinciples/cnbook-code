: Saturating STDP by Badoul et al (2006)
: Author: B. Graham, Computing Science & Maths, University of Stirling, U.K.
:  URL: www.cs.stir.ac.uk/~bpg/  Email: b.graham@cs.stir.ac.uk  
:  Last update: BPG 16-5-13

NEURON {
	POINT_PROCESS STDPB
	RANGE tau, e, i, d, p, dtau, ptau, wLTP, wLTD, wgt
	NONSPECIFIC_CURRENT i
}

UNITS {
	(nA) = (nanoamp)
	(mV) = (millivolt)
	(uS) = (microsiemens)
}

PARAMETER {
	tau = 0.1 (ms) <1e-9,1e9>
	e = 0	(mV)
	d = 0 <0,1>: depression factor
	p = 0 : potentiation factor
	dtau = 34 (ms) : depression effectiveness time constant
	ptau = 17 (ms) : Bi & Poo (1998, 2001)
	wLTP = 0	: maximum weight
	wLTD = 0	: minimum weight
}

ASSIGNED {
	v (mV)
	i (nA)
	tpost (ms)
	wgt (uS)
}

STATE {
	g (uS)
}

INITIAL {
	g=0
	tpost = -1e9
	net_send(0, 1)
}

BREAKPOINT {
	SOLVE state METHOD cnexp
	i = g*(v - e)
}

DERIVATIVE state {
	g' = -g/tau
}

NET_RECEIVE(w (uS), A, tpre (ms)) {
	INITIAL { A = 0  tpre = -1e9 }
	if (flag == 0) { : presynaptic spike  (after last post so depress)
		printf("entry flag=%g t=%g w=%g A=%g tpre=%g tpost=%g\n", flag, t, w, A, tpre, tpost)
		wgt = w + A
		g = g + w + A
		tpre = t
		A = A - (w+A-wLTD)*d*exp((tpost - t)/dtau)
	}else if (flag == 2) { : postsynaptic spike
		printf("entry flag=%g t=%g tpost=%g\n", flag, t, tpost)
		tpost = t
		FOR_NETCONS(w1, A1, tp) { : also can hide NET_RECEIVE args
			: printf("entry FOR_NETCONS w1=%g A1=%g tp=%g\n", w1, A1, tp)
			A1 = A1 + (wLTP-(w1+A1))*p*exp((tp - t)/ptau)
		}
	} else { : flag == 1 from INITIAL block
		: printf("entry flag=%g t=%g\n", flag, t)
		WATCH (v > -20) 2
	}
}