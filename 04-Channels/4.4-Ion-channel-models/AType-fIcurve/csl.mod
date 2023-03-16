TITLE csl.mod - Crustatean leak channel

COMMENT 

"Connor and Stevens" model for sodium channel from Connor,
Walter and McKown (Biophys. J. 18:81-102, 1977).

ENDCOMMENT

NEURON {
    SUFFIX csl
    NONSPECIFIC_CURRENT i
    RANGE i, g, e
}

PARAMETER {
    g = 0.0003 (S/cm2)
    e = -22 (mV)
}

ASSIGNED {
    v (mV)
    i (mA/cm2)
}

BREAKPOINT {
    i = g * (v - e)      
}
