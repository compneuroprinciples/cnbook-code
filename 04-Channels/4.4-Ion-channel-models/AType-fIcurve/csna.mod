TITLE cska.mod - Crustatean sodium channel

COMMENT 

"Connor and Stevens" model for sodium channel from Connor,
Walter and McKown (Biophys. J. 18:81-102, 1977).

ENDCOMMENT

NEURON {
    SUFFIX csna
    USEION na READ ena WRITE ina
    RANGE gbar, g, i
    GLOBAL Qten, Q
}

PARAMETER {
    gbar = 0.120 (S/cm2)
    Qten = 3.13                         : Adjusted to get the CS-factor of 
    celsius_exp = 6.3 (degC)            : Temperature at which
    : experiments performed
    mshift = -0.3  (mV)                 : Shift of -5.3 + 5 is CS shift and shift to match HH
    hshift = -7 (mV)                    : Shift of -12 + 5 is CS shift and shift to match HH
}

STATE {
    m h
}

ASSIGNED {
    v (mV)
    ena (mV)
    i (mA/cm2)
    ina (mA/cm2)
    g (S/cm2)
    celsius (degC)
    Q
}

BREAKPOINT {
    SOLVE states METHOD cnexp
    g = gbar * m^3 * h
    i = g * (v - ena)      
    ina = i 
}

INITIAL {
    m = alpham(v)/(alpham(v) + betam(v))
    h = alphah(v)/(alphah(v) + betah(v))
    Q = Qten^((celsius - celsius_exp)/10)
}

DERIVATIVE states {
    m' = Q * ((1 - m)*alpham(v) - m*betam(v))
    h' = Q * ((1 - h)*alphah(v) - h*betah(v))
}

FUNCTION alpham(Vm (mV)) (/ms) {
    alpham = 0.1 * vtrap(-(Vm + 35 + mshift), 10)
}

FUNCTION betam(Vm (mV)) (/ms) {
    betam = 4 * exp(-(Vm + 60 + mshift)/18)
}

FUNCTION alphah(Vm (mV)) (/ms) {
    alphah = 0.07*exp(-(Vm + 60 + hshift)/20)
}

FUNCTION betah(Vm (mV)) (/ms) {
    betah = 1/(exp(-(Vm + 30 + hshift)/10) + 1)
}

FUNCTION vtrap(x (mV) ,y (mV)) (mV) {  :Traps for 0 in denominator of rate eqns.
        if (fabs(x/y) < 1e-6) {
                vtrap = y*(1 - x/y/2)
        }else{
                vtrap = x/(exp(x/y) - 1)
        }
}
