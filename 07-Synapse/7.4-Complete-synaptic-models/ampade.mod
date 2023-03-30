TITLE detailed model of glutamate AMPA receptors

COMMENT
-----------------------------------------------------------------------------

	Kinetic model of AMPA receptors
	===============================

	6-state gating model:
	(scheme 1 from Raman and Trussell, Neuron 9:173-186, 1992)
        2 open states provide dual exponential response.
  
                     O1
                     |
	C ---- C1 -- C2 -- O2
	             |
      	             D

-----------------------------------------------------------------------------

  This mod file does not include mechanisms for the release and time course
  of transmitter; it is to be used in conjunction with a sepearate mechanism
  to describe the release of transmitter and that provides the concentration
  of transmitter in the synaptic cleft (to be connected to pointer C here).

  Default parameters are set for an mEPSC.

  Code based on Destexhe's ampa5.mod
  BPG 7-6-99

-----------------------------------------------------------------------------
ENDCOMMENT

INDEPENDENT {t FROM 0 TO 1 WITH 1 (ms)}

NEURON {
	POINT_PROCESS AMPADE
	POINTER C
	RANGE C0, C1, C2, D, O1, O2
	RANGE g, gmax, rb
	GLOBAL Erev
	GLOBAL Rb, Ru1, Ru2, Rd, Rr, Ro1, Rc1, Ro2, Rc2
	GLOBAL vmin, vmax
	NONSPECIFIC_CURRENT i
}

UNITS {
	(nA) = (nanoamp)
	(mV) = (millivolt)
	(pS) = (picosiemens)
	(umho) = (micromho)
	(mM) = (milli/liter)
	(uM) = (micro/liter)
}

PARAMETER {

	Erev	= 7    (mV)	: reversal potential
	gmax	= 20  (pS)	: maximal conductance
	vmin = -120	(mV)
	vmax = 100	(mV)
	
: Rates

	Rb	= 13   (/mM /ms): binding 
				: diffusion limited (DO NOT ADJUST)
	Ru1	= 0.3  (/ms)	: unbinding (1st site)
	Ru2	= 200  (/ms)	: unbinding (2nd site)		
	Rd	= 10.0   (/ms)	: desensitization
	Rr	= 0.02 (/ms)	: resensitization 
	Ro1	= 130    (/ms)	: opening (fast)
	Rc1	= 1.3    (/ms)	: closing
	Ro2	= 10    (/ms)	: opening (slow)
	Rc2	= 0.25    (/ms)	: closing
}

ASSIGNED {
	v		(mV)	: postsynaptic voltage
	i 		(nA)	: current = g*(v - Erev)
	g 		(pS)	: conductance
	C 		(mM)	: pointer to glutamate concentration
	rb		(/ms)	: binding
}

STATE {
	: Channel states (all fractions)
	C0		: unbound
	C1		: single glu bound
	C2		: double glu bound
 	D		: single glu bound, desensitized
	O1		: open state 1
	O2		: open state 2
}

INITIAL {
	C0=1
	C1=0
	C2=0
	D=0
	O1=0
	O2=0
}

BREAKPOINT {
	SOLVE kstates METHOD sparse

	g = gmax * (O1 + O2)
	i = (1e-6) * g * (v - Erev)
}

KINETIC kstates {
	
	rb = Rb * C 

	~ C0  <-> C1	(rb,Ru1)
	~ C1 <-> C2	(rb,Ru2)
	~ C2 <-> D	(Rd,Rr)
	~ C2 <-> O1	(Ro1,Rc1)
	~ C2 <-> O2	(Ro2,Rc2)

	CONSERVE C0+C1+C2+D+O1+O2 = 1
}

