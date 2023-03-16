TITLE cska.mod - Crustatean A-type potassium channel

COMMENT 

"Connor and Stevens" model for A-type potassium from Connor,
Walter and McKown (Biophys. J. 18:81-102, 1977).

ENDCOMMENT

NEURON {
    SUFFIX cska
    USEION k WRITE ik
    RANGE gbar, g, i
    GLOBAL e
}

PARAMETER {
    gbar = 0.0477 (S/cm2)
    e = -80 (mV)                        : CS minus 5mV to match HH
    shift = 5 (mV)
}

STATE {
    A B
}

ASSIGNED {
    v (mV)
    i (mA/cm2)
    ik (mA/cm2)
    g (S/cm2)
    celsius (degC)                      : These values valid at 18degC
}

BREAKPOINT {
    SOLVE states METHOD cnexp
    g = gbar * A^3 * B
    i = g * (v - e)
    ik =  i
}

INITIAL {
    A = Ainf(v)
    B = Binf(v)
}

DERIVATIVE states {
    A' =  (Ainf(v) - A)/tauA(v)
    B' =  (Binf(v) - B)/tauB(v)
}

FUNCTION Ainf(Vm (mV)) {
    Ainf = (0.0761*exp((Vm+shift+94.22(mV))/31.84(mV))/(1+exp((Vm+shift+1.17(mV))/28.93(mV))))^(1/3)
}

FUNCTION tauA(Vm (mV)) (ms) {
    tauA = 0.3632(ms)  + 1.158(ms)/(1 + exp((Vm+shift + 55.96(mV))/20.12(mV)))
}

FUNCTION Binf(Vm (mV)) {
    Binf = 1/(1 + exp((Vm+shift + 53.3(mV))/14.54(mV)))^4
}

FUNCTION tauB(Vm (mV)) (ms) {
    tauB = 1.24(ms) + 2.678(ms)/(1 + exp((Vm-shift+50(mV))/16.027(mV)))
}