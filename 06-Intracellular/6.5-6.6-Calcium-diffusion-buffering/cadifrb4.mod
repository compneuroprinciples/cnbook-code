TITLE Calcium ion accumulation and diffusion with kinetic buffering
: The internal coordinate system is set up in PROCEDURE coord()
: and must be executed before computing the concentrations.
: The scale factors set up in this procedure do not have to be recomputed
: when diam or DFree are changed.
: The amount of calcium in an annulus is ca[i]*diam^2*vol[i] with
: ca[0] being the second order correct concentration at the exact edge
: and ca[NANN-1] being the concentration at the exact center

: Version with fixed shell thickness.
: sthick is a range variable set from HOC.
: Rapid buffer approximation.
: Mods BPG 6-2-07

NEURON {
	SUFFIX cadifrb4
	USEION ca READ ica, cai WRITE cai
	GLOBAL vol
	RANGE cai0
}

DEFINE NANN  4

UNITS {
	(mV)	= (millivolt)
	(molar) = (1/liter)
	(mM)	= (millimolar)
	(um)	= (micron)
	(mA)	= (milliamp)
	(mol)	= (1)
	FARADAY = (faraday)	 (coulomb)
	PI	= (pi)		(1)
	R 	= (k-mole)	(joule/degC)
}

PARAMETER {
	celsius		(degC)
	diam		(um)
	cao		(mM)
	
	sthick = 0.1	(um)		: submembrane shell thickness
	DFree = .23	(um2/ms)	: diffusion coeff
	beta = 25			: buffering factor

	cai0 = 50e-6 (mM)	: Requires explicit use in INITIAL block
}

ASSIGNED {
	ica		(mA/cm2)
	cai		(mM)
	vol[NANN]	(1)	: gets extra cm2 when multiplied by diam^2
}

STATE {
	ca[NANN]	(mM) : ca[0] is equivalent to cai
}

INITIAL {LOCAL total
	if (sthick > diam/4) {
		sthick = diam/4	: limit outer shell thickness
	}
	coord()
	cai = cai0
	FROM i=0 TO NANN-1 {
		ca[i] = cai
	}
}

BREAKPOINT {
	SOLVE state METHOD sparse
}

LOCAL frat[NANN] 	: gets extra cm when multiplied by diam

PROCEDURE coord() {
	LOCAL r, dr2, drs
	: cylindrical coordinate system  with constant annuli thickness to
	: center of cell. Note however that the first annulus is half thickness
	: so that the concentration is second order correct spatially at
	: the membrane or exact edge of the cell.
	: note ca[0] is at edge of cell
	:      ca[NANN-1] is at center of cell
	r = 1/2					:starts at edge (half diam)
	drs = sthick/diam			:relative shell thickness
	dr2 = (r-drs)/(2*NANN-3)		:half thickness of annulus
	vol[0] = PI*(r-drs/2)*2*drs
	frat[0] = 2*r
	r = r - drs
	frat[1] = 2*PI*r/(drs+dr2)
	r = r - dr2
	vol[1] = PI*(r+dr2/2)*2*dr2	:outer half of annulus
	FROM i=1 TO NANN-2 {
		vol[i] = vol[i] + PI*(r-dr2/2)*2*dr2	:interior half
		r = r - dr2
		frat[i+1] = 2*PI*r/(2*dr2)	:exterior edge of annulus
					: divided by distance between centers
		r = r - dr2
		vol[i+1] = PI*(r+dr2/2)*2*dr2	:outer half of annulus
	}
}

LOCAL dsq, dsqvol	: can't define local variable in KINETIC block 
			: or use in COMPARTMENT

KINETIC state {
	COMPARTMENT i, (1+beta)*diam*diam*vol[i]*1(um) {ca}

	: all currents except pump
	~ ca[0] << (-ica*PI*diam*1(um)*(1e4)*frat[0]/(2*FARADAY))
	:diffusion
	FROM i=0 TO NANN-2 {
		~ ca[i] <-> ca[i+1] (DFree*frat[i+1]*1(um), DFree*frat[i+1]*1(um))
	}
	cai = ca[0]
}
	
FUNCTION ss() (mM) {
	SOLVE state STEADYSTATE sparse
	ss = cai
}
