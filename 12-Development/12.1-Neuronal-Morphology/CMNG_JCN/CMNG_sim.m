function CMNG_sim(sim_params, mod_params, calc_params, plots)
% CMNG_sim Simulation of Continuum Model for Neurite Outgrowth
% Continuum Model for Neurite Outgrowth
% Version 1.0 (BPG & DRM 25-10-05)

% Run model for jmax time steps
[C, C0, CN, l] = CMNG_run(sim_params, mod_params, calc_params, -1, mod_params);
[t, C, C0, CN, l] = CMNG_dimen(sim_params, mod_params, C, C0, CN, l);  % dimensionalise
Ca = [C0 C CN];

% Plot results
i=0:length(l)-1;
t=i*datat*dt;

figure(1);
subplot(2,2,1);
plot(t,l);
hold on;
title('Length');
xlabel('Time');
ylabel('Length');

subplot(2,2,2);
%figure;
plot(t,CN);
hold on;
plot(t,C0,'g-');
hold on;
title('Concentration');
xlabel('Time');
ylabel('Concentration');
legend('GC','Soma',3);

subplot(2,2,3);
%figure;
X=0:N;
Y=0:datat:jmax;
Call = [C0 C CN];
surf(X,Y,Call);
title('Concentration over Unit Space');
xlabel('Space');
ylabel('Time');
zlabel('Concentration');
view(90-37.5,30);

subplot(2,2,4);
%figure;
space=0:1/N:1;
X=zeros(floor(jmax/datat)+1,N+1);
for j=1:length(l)
    X(j,:)=l(j)*space;
end
Y=0:datat:jmax;
Call = [C0 C CN];
surf(X,Y,Call);
title('Concentration over Length');
xlabel('Length');
ylabel('Time');
zlabel('Concentration');
view(90-37.5,30);
