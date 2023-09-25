TITLE Three compartment model of calcium concentration

COMMENT
Internal calcium accumulation ([Ca]i) with 3 compartment model.
Consists of a submembrane shell and a core shell, with the submembrane
shell being subdivided into two compartments, corresponding to membrane
surrounding calcium channels, and the remaining membrane, respectively.

Adapted from Lyle J. Borg-Graham, Interpretations of data and mechanisms
for hippocampal pyramidal cell models.  In "Cerebral Cortex, Vol 13:
Cortical Models", Plenum Press 1998

Implemented by BPG 14-6-06
ENDCOMMENT

NEURON {
	SUFFIX ca3c
	USEION ca READ cai, ica WRITE cai
	RANGE ca2i,chflux,pflux,Jleak,df12,df21,df23,df32
	GLOBAL DFree,rshell,alpha1,alpha12,Vmax,kd,bBu,r12,r23
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
	refcai=5e-5	(mM)	: cai for setting Jleak
	rshell=1	(um)	: thickness of juxtamembrane shell
	alpha1=1e-4	(1)	: colocation fraction
	alpha12=1	(/um)	: interdigitation coefficient
	DFree=0.6	(um2/ms)	: diffusion coeff
	r12=1		(um)	: distance between comps. 1 and 2
	r23=1		(um)	: distance between comps. 2 and 3
	Vmax=9e-8	(mmol/ms-cm2)	: De Schutter
	kd=0.001	(mM)		: De Schutter
	bBu=20		(1)
}

ASSIGNED {
	diam		(um)
	cai		(mM)	: free calcium in compartment 1
	ca2i		(mM)	: free calcium in compartment 2
	a23		(um)	: cross-sectional area (per unit length)
	v2		(um2)	: submembrane shell volume (per unit length)
	v3		(um2)	: core shell volume (per unit length)
	df12		(/ms)	: diffusional flux between comps.1 & 2
	df21		(/ms)	: etc
	df23		(/ms)
	df32		(/ms)
	chflux		(mM/ms)
	pflux		(mmol/ms-cm2)
	Jleak		(mmol/ms-cm2)
}

STATE {
	ca1		(mM) : free ca conc. in compartment 1
	ca2		(mM) : free ca conc. in compartment 2
	ca3		(mM) : free ca conc. in compartment 3
}

INITIAL {
: check and alter dimensions as per segment diameter
	if (rshell > diam/4) {rshell = diam/4}	: restrict to half radius
	r12 = PI*diam/2 	: max half circumference
	r23 = diam/2 		: membrane to centre (radius)
	a23 = 2*(r23-rshell)	: cross-sectional area between shells
	v2 = rshell*(2*r23-rshell)	: volume of submembrane shell
	v3 = (r23-rshell)*(r23-rshell)	: volume of core shell
	df12 = DFree*alpha12/((1-alpha1)*r12)
	df21 = DFree*alpha12/(alpha1*r12)
	df23 = DFree*a23*(1-alpha1)/(r23*v3)
	df32 = DFree*a23/(r23*v2)
VERBATIM
	cai = _ion_cai;
ENDVERBATIM
	ca2i = cai
	ca1 = cai
	ca2 = cai
	ca3 = cai
	chflux = 0
	pflux = 0
	Jleak = Vmax*refcai/(refcai+kd)
}

BREAKPOINT {
	SOLVE state METHOD derivimplicit
}

DERIVATIVE state { 

        chflux = -ica/(2*FARADAY*alpha1*rshell)	: channel influx

        pflux = -(Vmax*ca2/(ca2+kd))+Jleak  : pump eflux

        ca1' = (chflux + (df21*(ca2-ca1)))/(bBu+1)
        ca2' = (((1e7)*pflux/rshell) + df12*(ca1-ca2) + df32*(ca3-ca2))/(bBu+1)
	ca3' = df23*(ca2-ca3)/(bBu+1)

	cai = ca1	: Ca(i) for Ca channels and Ict is conc. in comp.1
	ca2i = ca2	: Ca(i) for other channels is conc. in comp.2
}

	
COMMENT
At this time, conductances (and channel states and currents are
calculated at the midpoint of a dt interval.  Membrane potential and
concentrations are calculated at the edges of a dt interval.  With
secondorder=2 everything turns out to be second order correct.
ENDCOMMENT

