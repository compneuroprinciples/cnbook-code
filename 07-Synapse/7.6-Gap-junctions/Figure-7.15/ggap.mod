COMMENT
Gap junction model

As specified in Example 10.2 of the NEURON book.

Junction is Ohmic and specified by a junctional resistance, r.

To create a single gap junction, two instances of this point process
need to be inserted, one in each compartment of the joined cells. The
pointer is then set to point to the voltage in the compartment of the
other cell.

BPG 15-11-07
ENDCOMMENT

NEURON {
	POINT_PROCESS Ggap
	POINTER vgap
	RANGE r, i
	NONSPECIFIC_CURRENT i
}

UNITS {
	(nA) = (nanoamp)
	(mV) = (millivolt)
	(umho) = (micromho)
}

PARAMETER {
	r=1e10 (megohm)
}

ASSIGNED {
	v (mV)
	i (nA)
	vgap (mV)
}


BREAKPOINT {
	i = (v - vgap) / r
}

