: stochstim.mod
: Stimulus train for Netcons, with stochastic release and vesicle recycling.
: Implements vesicle-state model.
: Adapted from netstim.mod
: BPG 4-1-07

NEURON	{ 
  ARTIFICIAL_CELL StochStim
  RANGE interval, number, start, noise
  RANGE p0, p1, tauf, kr, n0
  RANGE pv, n
}

PARAMETER {
	interval	= 10 (ms) <1e-9,1e9>: time between spikes (msec)
	number	= 10 <0,1e9>	: number of spikes (independent of noise)
	start		= 50 (ms)	: start of first spike
	noise		= 0 <0,1>	: amount of randomness (0.0 - 1.0)
	p0 = 0.2	: initial release probability
	p1 = 0.05	: release increment
	tauf = 100 (ms)	: recovery from facilitation
	kr = 0.0002 (/ms)	: background undocking rate
	n0 = 1		: initial size of RRVP
}

ASSIGNED {
	event (ms)
	on
	ispike
	tprev (ms)	: previous presynaptic spike time
	isi (ms)	: interspike interval
	release_event	: flag a release has actually occurred
	pv		: release probability
	n		: size of RRVP
	kn (/ms)	: background replenishment rate
}

PROCEDURE seed(x) {
	set_seed(x)
}

INITIAL {
	on = 0
	ispike = 0
	if (noise < 0) {
		noise = 0
	}
	if (noise > 1) {
		noise = 1
	}
	if (start >= 0 && number > 0) {
		: randomize the first spike so on average it occurs at
		: start + noise*interval
		event = start + invl(interval) - interval*(1. - noise)
		: but not earlier than 0
		if (event < 0) {
			event = 0
		}
		net_send(event, 3)
	}
	release_event = 0
	pv = p0
	n = n0
	kn = n * kr
	tprev = 0
}	

PROCEDURE init_sequence(t(ms)) {
	if (number > 0) {
		on = 1
		event = 0
		ispike = 0
	}
}

FUNCTION invl(mean (ms)) (ms) {
	if (mean <= 0.) {
		mean = .01 (ms) : I would worry if it were 0.
	}
	if (noise == 0) {
		invl = mean
	}else{
		invl = (1. - noise)*mean + noise*mean*exprand(1)
	}
}

PROCEDURE next_invl() {
	if (number > 0) {
		event = invl(interval)
	}
	if (ispike >= number) {
		on = 0
	}
}

PROCEDURE release() {
	: stochastic vesicle release and recycling (BPG 4-1-07)
	release_event = 0
	isi = t - tprev
	: release facilitation
	pv = pv + p1*(1-pv)
	pv = pv + (1-exp(-isi/tauf))*(p0-pv)
	: vesicle recycling assumes only a single new vesicle is likely
	: to arrive during a single ISI (not true if kn or isi are large)
	if (unirand()<=kn*isi) {	: vesicle arrival from reserve pool
	  n = n+1
	}
	FROM i=1 TO n {
	  if (unirand()<=kr*isi) {	: vesicle undocking
	    n = n-1
	  }
	}
	FROM i=1 TO n {
	  if (release_event == 0) {	: allow only one release per spike
	    if (unirand()<=pv) {		: vesicle release
	      n = n-1
	      release_event = 1
	    }
	  }
	}
}

NET_RECEIVE (w) {
	if (flag == 0) { : external event
		if (w > 0 && on == 0) { : turn on spike sequence
			init_sequence(t)
			: randomize the first spike so on average it occurs at
			: noise*interval (most likely interval is always 0)
			next_invl()
			event = event - interval*(1. - noise)
			net_send(event, 1)
		}else if (w < 0 && on == 1) { : turn off spiking
			on = 0
		}
	}
	if (flag == 3) { : from INITIAL
		if (on == 0) {
			init_sequence(t)
			net_send(0, 1)
		}
	}
	if (flag == 1 && on == 1) {
		ispike = ispike + 1
		release(t)
		tprev = t
		if (release_event == 1) {
			net_event(t)
		}
		next_invl()
		if (on == 1) {
			net_send(event, 1)
		}
	}
}

FUNCTION unirand() {    : uniform random numbers between 0 and 1
        unirand = scop_random()
}

COMMENT
Presynaptic spike generator
---------------------------

This mechanism has been written to be able to use synapses in a single
neuron receiving various types of presynaptic trains.  This is a "fake"
presynaptic compartment containing a spike generator.  The trains
of spikes can be either periodic or noisy (Poisson-distributed)

Parameters;
   noise: 	between 0 (no noise-periodic) and 1 (fully noisy)
   interval: 	mean time between spikes (ms)
   number: 	number of spikes (independent of noise)

Written by Z. Mainen, modified by A. Destexhe, The Salk Institute

Modified by Michael Hines for use with CVode
The intrinsic bursting parameters have been removed since
generators can stimulate other generators to create complicated bursting
patterns with independent statistics (see below)

Modified by Michael Hines to use logical event style with NET_RECEIVE
This stimulator can also be triggered by an input event.
If the stimulator is in the on=0 state and receives a positive weight
event, then the stimulator changes to the on=1 state and goes through
its entire spike sequence before changing to the on=0 state. During
that time it ignores any positive weight events. If, in the on=1 state,
the stimulator receives a negative weight event, the stimulator will
change to the off state. In the off state, it will ignore negative weight
events. A change to the on state immediately fires the first spike of
its sequence.

ENDCOMMENT

