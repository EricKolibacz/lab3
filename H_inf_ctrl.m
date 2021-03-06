%% This is a Matlab file for designing H_infinity controller (assignment 3
%% of SD2231)
clear all
close all
s=tf('s');

% systme parameters
m=22000;   %kg
j=700e3;   %kgm^2
J=700e3;   %kgm^2
c=40e3;    %Ns/m
k=2*300e3; %N/m
L=6;       %m

%% State space model for skyhook contorl
Ask=[0 1 0 0
    -2*k/m 0 0 0
    0 0 0 1
    0 0 -2*k*L^2/j 0];
Bsk=[0 0 0 0
    k/m k/m -1/m -1/m
    0 0 0 0
    -L*k/j L*k/j L/j -L/j];
Csk=[0 1 0 0
    0 0 0 1];
Dsk=zeros(2,4);

G = ss(Ask,Bsk,Csk,Dsk);

%bode(G)



%% H_inf using linmod syntax

%state space: The same as skyhook

      
%Weighting functions
close all
%For penalizing actuator force
Wa1=(0.00175*s+1)/(0.00025*s+1);
Wa2=Wa1;

%For penalizing bounce and pitch motions
eps=1;
wnb=7.39;            %Find the right equation or value for wnb
wnchi=7.39;          %Find the right equation or value for wnchi
s1b=-eps+1i*sqrt(wnb^2-eps^2);
s2b=-eps-1i*sqrt(wnb^2-eps^2);
s1chi=-eps+1i*sqrt(wnchi^2-eps^2);
s2chi=-eps-1i*sqrt(wnchi^2-eps^2);
kb=6.66*10^3%input('Enter the gain for Wb = '); % around 10^-3
kchi=4.66*10^4%input('Enter the gain for Wchi = '); % around 10^4
Wb=(kb*s1b*s2b)/((s-s1b)*(s-s2b));
Wchi=(kchi*s1chi*s2chi)/((s-s1chi)*(s-s2chi));

%Extracting the extended model
[A_Pe,B_Pe,C_Pe,D_Pe] = linmod('Extended_model');% state space parameters of the extended system: Pe
Pe=ss(A_Pe,B_Pe,C_Pe,D_Pe);

%Calculating the controller
ncont = 2;%Number of control inputs
nmeas = 2;%Number of measured outputs provided to the controller
Pe=minreal(Pe);%This syntax cancels pole-zero pairs in transfer
%functions. The output system has minimal order and the same response
%characteristics as the original model.
[K,Pec,gamma,info]=hinfsyn(Pe,nmeas,ncont,'method','lmi'); % for working with the error

[Ainf, Binf, Cinf, Dinf]=ssdata(K);

%Now use the controller K in your simulation

exictation = 0;
f = 1;
run_time = 6;
sim('Hinf_sim')




A = [0 1 0 0;
    -2*k/m -2*c/m 0 0;
    0 0 0 1;
    %0 0 -2*L/J -2*L/J];
    0 0 -2*k*L^2/J -2*c*L^2/J];
B = [0 0 0 0;
    k/m c/m k/m c/m;
    0 0 0 0;
    %-L1*k1/J -L1*c1/J +L2*k2/J +L2*c2/J];
    -L*k/J -L*c/J L*k/J L*c/J];
C = [1 0 0 0;
    0 0 1 0];
D = [0 0 0 0
    0 0 0 0];



G_ss = ss(A,B,C,D);

sim chassis_passive_sim


figure(1)
plot(z_pas.Time,z_pas.Data,'Color','r','LineWidth',1.5,'DisplayName','Y1 for passive');
hold on;
plot(z_Hinf.Time,z_Hinf.Data,'Color','b','LineWidth',1.5,'DisplayName','Y1 for Hinf');
legend('show','Location','NorthEast');
xlabel('Time [sec]');
ylabel('Displacement [m]');
grid on;
figure(2);
plot(X_pas.Time,X_pas.Data,'Color','r','LineWidth',1.5,'DisplayName','Y2 for passive');
hold on;
plot(X_Hinf.Time,X_Hinf.Data,'Color','b','LineWidth',1.5,'DisplayName','Y2 for Hinf');
legend('show','Location','NorthEast');
xlabel('Time [sec]');
ylabel('Angle [rad]');
grid on;

figure(3)
bode(Wa1)
title('bode of Wa1, Wa2')

figure(4)
bode(Wb)
title('bode of Wb')

figure(5)
bode(Wchi)
title('bode of Wchi')

%Fa1 Fa2
figure(6);
plot(Fa1.Time,Fa1.Data,'Color','r','LineWidth',1.5,'DisplayName','Fa1');
hold on;
plot(Fa2.Time,Fa2.Data,'Color','b','LineWidth',1.5,'DisplayName','Fa2');
legend('show','Location','NorthEast');
xlabel('Time [sec]');
ylabel('Force [N]');
grid on;
