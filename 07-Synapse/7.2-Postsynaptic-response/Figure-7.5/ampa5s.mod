TITLE Kinetic model of glutamate AMPA receptors

COMMENT
-----------------------------------------------------------------------------

	Kinetic model of AMPA receptors
	===============================

	5-state gating model:
  
	C ---- C1 -- C2 -- O
	             |
      	             D

-----------------------------------------------------------------------------

  Default parameters are set for an mEPSC.

  Code based on Destexhe's ampa5.mod
  BPG 17-11-06

-----------------------------------------------------------------------------
ENDCOMMENT

INDEPENDENT {t FROM 0 TO 1 WITH 1 (ms)}

NEURON {
	POINT_PROCESS AMPA5s
	RANGE C0, C1, C2, D, O
	RANGE g, gmax, rb
	GLOBAL Erev, Camp, Cdur
	GLOBAL Rb, Ru1, Ru2, Rd, Rr, Ro, Rc
	NONSPECIFIC_CURRENT i
}

UNITS {
	(nA) = (nanoamp)
	(mV) = (millivolt)
	(nS) = (nanosiemens)
	(umho) = (micromho)
	(mM) = (milli/liter)
	(uM) = (micro/liter)
}

PARAMETER {

	Erev	= 7    (mV)	: reversal potential
	gmax	= 1  (nS)	: maximal conductance
: Transmitter
	Camp	= 1 (mM)	: amplitude
	Cdur	= 1 (ms)	: duration
	
: Rates

	Rb	= 13   (/mM /ms): binding 
				: diffusion limited (DO NOT ADJUST)
	Ru1	= 0.3  (/ms)	: unbinding (1st site)
	Ru2	= 200  (/ms)	: unbinding (2nd site)		
	Rd	= 10.0   (/ms)	: desensitization
	Rr	= 0.02 (/ms)	: resensitization 
	Ro	= 100    (/ms)	: opening (fast)
	Rc	= 1.0    (/ms)	: closing
}

ASSIGNED {
	v		(mV)	: postsynaptic voltage
	i 		(nA)	: current = g*(v - Erev)
	g 		(nS)	: conductance
	C 		(mM)	: pointer to glutamate concentration
	rb		(/ms)	: binding
}

STATE {
	: Channel states (all fractions)
	C0		: unbound
	C1		: single glu bound
	C2		: double glu bound
 	D		: single glu bound, desensitized
	O		: open state
}

INITIAL {
	C0=1
	C1=0
	C2=0
	D=0
	O=0
}

BREAKPOINT {
	SOLVE kstates METHOD sparse

	g = gmax * O
	i = (1e-3) * g * (v - Erev)
}

KINETIC kstates {
	
	rb = Rb * C 

	~ C0  <-> C1	(rb,Ru1)
	~ C1 <-> C2	(rb,Ru2)
	~ C2 <-> D	(Rd,Rr)
	~ C2 <-> O	(Ro,Rc)

	CONSERVE C0+C1+C2+D+O = 1
}

NET_RECEIVE(weight, on) {

	if (flag == 0) {
		: spike arrived
		if (!on) {
			C = Camp
			on = 1
			: event in Cdur msecs, with flag=1
			net_send(Cdur, 1)
		} else {
		: already in onset state, so move offset time
			net_move(t+Cdur)
		}
	}
	if (flag == 1) {
		: internal event to turn off transmitter
		C = 0
		on = 0
	}
}


