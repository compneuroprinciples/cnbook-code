function [Cinf, linf] = CMNG_lCanal(simp, modp, calcp, lsim)
% CMNG_lCanal: Function to calculate steady-state length and concentrations
% analytically, if possible
% Continuum Model for Neurite Outgrowth
% Version 2.1 (BPG & DRM 17-6-03)

Cinf = zeros(1, simp.N+1);
y = 0:simp.N;
y = y./simp.N;

altwid = calcp.alpha*calcp.gamma/calcp.phi;
phitwid = calcp.phi/(calcp.gamma*sqrt(calcp.beta));

% Degenerate case I
if (calcp.alpha == 0 & calcp.beta > 0)
    linf = (1/sqrt(calcp.beta))*log(phitwid+sqrt(phitwid^2+1));
    Cinf = calcp.gamma*cosh(linf*sqrt(calcp.beta)*(1-y));

% Degenerate case II
elseif (calcp.alpha > 0 & calcp.beta == 0)
    linf = 0;
    disp 'No steady-state solution'
    
% Asymptotic case
elseif (calcp.alpha > 0 & calcp.beta > 0 & calcp.beta < calcp.alpha^2)
    h = calcp.beta/calcp.alpha^2;
    alh = altwid*h;
    if (alh > 1)
        Linf0 = log(alh/(alh-1));
        Linf1 = (2/(alh-1))-(1+alh/(alh-1))*Linf0;
        Linf = Linf0 + h*Linf1;
    else
        Linfm1 = log(1/alh);
        Linf0 = 2 + Linfm1;
        Linf = Linfm1/h + Linf0;
    end
    linf = Linf/calcp.alpha;
    fhp = 0.5*(1+sqrt(1+4*h));
    fhm = 0.5*(1-sqrt(1+4*h));
    D = h*(exp(Linf*fhm)-exp(Linf*fhp));
    f1 = calcp.phi/(calcp.alpha*D);
    A = -f1*fhp*exp(Linf*fhp);
    B = f1*fhm*exp(Linf*fhm);
    Cinf = A*exp(Linf*fhm*y) + B*exp(Linf*fhp*y);
        
% No analytical solution derived or possible
else
    linf = 0;
    disp 'Case not covered'
end
        
