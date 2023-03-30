: based on ExpSyn
: PARAMETERs sid and cid are used to manage and decipher network architecture

NEURON {
:	POINT_PROCESS ExpSyn
	POINT_PROCESS ExpSid
	RANGE tau, e, i, sid, cid
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
	: following are used to help decipher and manage network architecture 
	sid = -1 (1) : synapse id, from cell template
	cid = -1 (1) : id of cell to which this synapse is attached
	             : in parallel context this value should be assigned the cell's gid
}

ASSIGNED {
	v (mV)
	i (nA)
}

STATE {
	g (uS)
}

INITIAL {
	g=0
}

BREAKPOINT {
	SOLVE state METHOD cnexp
	i = g*(v - e)
}

DERIVATIVE state {
	g' = -g/tau
}

NET_RECEIVE(weight (uS)) {
	state_discontinuity(g, g + weight)
}
