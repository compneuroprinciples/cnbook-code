TITLE Ca concentration in submembrane and core shells.

COMMENT
Internal calcium accumulation ([Ca]i) in 2 pool model.
Pools consist of a thin submembrane shell, and the remaining core
of the cell compartment. Submembrane concentration is calculated at
the submembrane surface.
Instantaneous buffering included.

Implemented by BPG 19-9-07
ENDCOMMENT

NEURON {
	SUFFIX ca2p
	USEION ca READ cai, ica WRITE cai
	RANGE ca2i, cai0, rshell
	GLOBAL DFree,bBu
}


INDEPENDENT {t FROM 0 TO 1 WITH 10 (ms)}

UNITS {
        (mol)   = (1)
        (mmol)   = (millimol)
	(molar) = (1/liter)
	(mM)	= (millimolar)
	(um)	= (micron)
	(mA)	= (milliamp)
	FARADAY = (faraday)	 (10000 coulomb)
	PI	= (pi) (1)
}

PARAMETER {
	ica		(mA/cm2)
	cai0=5e-5	(mM)	: cai for setting Jleak
	rshell=1	(um)	: thickness of juxtamembrane shell
	DFree=0.23	(um2/ms)	: diffusion coeff
	r12=10		(um)	: distance between shells 1 and 2
	bBu=20		(1)
}

ASSIGNED {
	diam		(um)
	cai		(mM)	: free calcium in shell 1 (submembrane)
	ca2i		(mM)	: free calcium in shell 2 (core)
	a12		(um)	: cross-sectional area (per unit length)
	v1		(um2)	: submembrane shell volume (per unit length)
	v2		(um2)	: core shell volume (per unit length)
	df12		(/ms)	: diffusional flux between shells 1 & 2
	df21		(/ms)	: etc
	chflux		(mM/ms)
}

STATE {
	ca1		(mM) : free ca conc. in shell 1
	ca2		(mM) : free ca conc. in shell 2
}

INITIAL {
: check and alter dimensions as per segment diameter
	if (rshell > diam/4) {rshell = diam/4}	: restrict to half radius
	r12 = diam/2 	: distance to centre (from membrane)
	a12 = 2*(r12-rshell)	: cross-sectional area between shells
	v1 = rshell*(2*r12-rshell)	: volume of submembrane shell
	v2 = (r12-rshell)*(r12-rshell)	: volume of core shell
	df12 = DFree*a12/(r12*v2)
	df21 = DFree*a12/(r12*v1)
	cai = cai0
	ca2i = cai
	ca1 = cai
	ca2 = cai
	chflux = 0
}

BREAKPOINT {
	SOLVE state METHOD derivimplicit
}

DERIVATIVE state { 

        chflux = -ica*diam/(2*FARADAY*v1)	: channel influx

        ca1' = (chflux + df21*(ca2-ca1))/(bBu+1)
	ca2' = df12*(ca1-ca2)/(bBu+1)

	cai = ca1	: Ca(i) for Ca channels and Ica is conc. in shell.1
	ca2i = ca2
}

	
COMMENT
At this time, conductances (and channel states and currents are
calculated at the midpoint of a dt interval.  Membrane potential and
concentrations are calculated at the edges of a dt interval.  With
secondorder=2 everything turns out to be second order correct.
ENDCOMMENT

