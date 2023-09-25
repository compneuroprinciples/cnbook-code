TITLE Instantaneous Michaelis-Menten membrane-bound calcium pump.

COMMENT
Pump flux also contributes to the total calcium current.

Implemented by BPG 4-11-06
ENDCOMMENT

NEURON {
	SUFFIX caJpump
	USEION ca READ cai WRITE ica
	RANGE pflux,Jleak, cai0
	GLOBAL Vmax,kd
}

UNITS {
	(mA) = (milliamp)
	(um) = (micron)
	(mol) = (1)
        (mmol)   = (millimol)
	(molar) = (1/liter)
	(mM)	= (millimolar)
	PI = (pi) (1)
	FARADAY = (faraday) (coulomb)
}

STATE {
	pump	(mol/cm2)
	pumpca	(mol/cm2)
	cai	(mM)
}

PARAMETER {
	diam		(um)
	Vmax=4e-11	(mmol/ms-cm2)
	kd=0.01	(mM)
	cai0 = 50e-6 (mM)	: Requires explicit use in INITIAL block
}

ASSIGNED {
	ica 		(mA/cm2)
	ipump 		(mA/cm2)
	pflux		(mmol/ms-cm2)
	Jleak		(mmol/ms-cm2)
}

INITIAL {
	cai = cai0
	pflux = 0
	Jleak = Vmax*cai/(cai+kd)
}

BREAKPOINT {
        pflux = (Vmax*cai/(cai+kd))-Jleak  	: pump eflux
	ipump = (1e+3)*2*FARADAY*pflux
	ica = ipump
}
