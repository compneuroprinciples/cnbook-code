TITLE cska.mod - Crustatean delayed rectifier potassium channel

COMMENT 

"Connor and Stevens" model for sodium channel from Connor,
Walter and McKown (Biophys. J. 18:81-102, 1977).

ENDCOMMENT

NEURON {
    SUFFIX csk
    USEION k READ ek WRITE ik
    RANGE gbar, g, i, n
    GLOBAL Q
}

PARAMETER {
    gbar = 0.020 (S/cm2)
    Qten = 3.13                         : Adjusted to get the CS-factor of 
    celsius_exp = 6.3 (degC)            : Temperature at which
    : experiments performed
    nshift = 0.7 (mV)                   :  CS shift and shift to match HH  =-4.3 + 5 
}

STATE {
    n
}

ASSIGNED {
    v (mV)
    ek (mV)
    i (mA/cm2)
    ik (mA/cm2)
    g (S/cm2)
    celsius (degC)
    Q
}

BREAKPOINT {
    SOLVE states METHOD cnexp
    g = gbar * n^4
    i = g * (v - ek)
    ik = i 
}

INITIAL {
    n = alphan(v)/(alphan(v) + betan(v))
    Q = Qten^((celsius - celsius_exp)/10)
}

DERIVATIVE states {
    n' = Q / 2 * ((1 - n)*alphan(v) - n*betan(v))
}

FUNCTION alphan(Vm (mV)) (/ms) {
    alphan = 0.01 * vtrap(-(Vm + 50 + nshift), 10)
}

FUNCTION betan(Vm (mV)) (/ms) {
    betan = 0.125 * exp(-(Vm + 60 + nshift)/80)
}

FUNCTION vtrap(x (mV) ,y (mV)) (mV) {  :Traps for 0 in denominator of rate eqns.
        if (fabs(x/y) < 1e-6) {
                vtrap = y*(1 - x/y/2)
        }else{
                vtrap = x/(exp(x/y) - 1)
        }
}
