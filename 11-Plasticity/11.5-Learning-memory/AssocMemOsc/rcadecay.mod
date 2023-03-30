: $Id: rcadecay.mod,v 1.1 2009/01/19 16:39:52 sterratt Exp $
TITLE rcadecay

INDEPENDENT {t FROM 0 TO 1 WITH 10 (ms)}

NEURON {
	SUFFIX rcadecay
	USEION ca READ ica WRITE cai
	RANGE  phi, beta
}

UNITS {
	(molar) = (1/liter)
	(mM)	= (millimolar)
	(um)	= (micron)
	(mA)	= (milliamp)
}

PARAMETER {
	phi             = 0.13 
	beta            = 0.075
}
ASSIGNED {
  ica
}

STATE {	cai (1) }

INITIAL {
  cai=0.0
}

BREAKPOINT {
  if       ( cai < 0 )      { cai = 0 
  } else                    { SOLVE state METHOD cnexp }
}

DERIVATIVE state {
  cai' = - phi * ica - beta * cai 
}
