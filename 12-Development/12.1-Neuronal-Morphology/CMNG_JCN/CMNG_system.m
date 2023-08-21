function [A, b] = CMNG_system(C, C0, CN, CTD0, l, lk, simp, modp, calcp)
% CMNG_system: Function to build system matrix A and vector b
% C = concentration at previous time step
% C0 = conc. at previous time step at proximal end
% CN = conc. at previous time step at distal end
% l = length at previous time step
% lk = length at current corrector step
% Continuum Model for Autoregulatory-Time Delay Neurite Outgrowth
% Version 2.0 (DRM & BPG 19-7-04)
% Version 2.0.2 (BPG & DRM 31-1-03)

% Parameters
N = simp.N;
dt = simp.dt;
dy = calcp.dy;

% System matrix

A = zeros(N-1); % initialise system matrix

f1 = (dy*lk)^2;

A(1,1) = (dt/3)+f1;
A(1,2) = -dt/3;
A(N-1,N-1) = (dt/3)+f1;
A(N-1,N-2) = -dt/3;

for i=2:N-2
    A(i,i) = dt+f1;
    A(i,i-1) = -dt/2;
    A(i,i+1) = -dt/2;
end


% System vector

b = zeros(N-1,1); % initialise system vector

f1 = ((dt*lk*lk)/(2*l));
f2 = dy*dy*(CN-calcp.gamma);
fP = (1/l)+(calcp.alpha*dy);
fR = (1/l)-(calcp.alpha*dy);
Q = (lk*lk)*(((dy*dy)*(1-(calcp.beta*dt)))-(dt/(l*l)));
%theta1 = dt*dy*calcp.phi*l*(1-calcp.theta*C0)/3;
theta1 = dt*dy*calcp.phi*l*(1-calcp.theta*CTD0)/3;
thetaN = -theta1*((calcp.rho*CN)-calcp.sigma);
P = f1*(fP-f2); 
R = f1*(fR+f2);
b(1) = (P*C0)+(Q*C(1))+(R*C(2))+theta1;
P = f1*(fP-((N-1)*f2)); 
R = f1*(fR+((N-1)*f2));
b(N-1) = (P*C(N-2))+(Q*C(N-1))+(R*CN)+thetaN;
for i=2:N-2
    P = f1*(fP-(i*f2)); 
    R = f1*(fR+(i*f2));
    b(i) = (P*C(i-1))+(Q*C(i))+(R*C(i+1));
end
