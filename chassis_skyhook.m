%% Simple Vehicle model
% Comparison Skyhook and passive control

clear all;
close all;


%% paramters
s = tf('s');
m = 22000;
J = 700000;
c = 40000;
c1 = c;
c2 = c;
k = 600000;
k1 = k;
k2 = k;
L = 6;
L1 = L;
L2 = L;

%Tuned parameter for the actuators
c_z = 66666;
c_X = 66666*35;


%% Tunable parameters
%Excitation 0=sinusiodal, 1 = step
exictation = 0;
%Frequency in Hz
f = 8;

run_time = 20;



%% Passive system
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


sim chassis_passive_sim
%At this point z_pas and X_pas are defined


%% Skyhook system
A = [0 1 0 0;
    -2*k/m 0 0 0;
    0 0 0 1;
    0 0 -2*k*L^2/J 0];
B = [0 0 0 0;
    k/m k/m 1/m 1/m;
    0 0 0 0;
    -k*L/J k*L/J -L/J L/J];
C = [0 1 0 0;
    0 0 0 1];
D = [0 0 0 0
    0 0 0 0];



G_ss = ss(A,B,C,D);


sim chassis_skyhook_sim
%At this point z_SH and X_SH should be defined
%as well as the two forces Fa1 and Fa2

%% Plots

%two figures of the output variables as comparison between passive and
%Skyhook
plot(z_SH.Time,z_SH.Data,'Color','r','LineWidth',1.5,'DisplayName','Y1 for Skyhook');
hold on;
plot(z_pas.Time,z_pas.Data,'Color','b','LineWidth',1.5,'DisplayName','Y1 for passive');
legend('show','Location','NorthEast');
xlabel('Time [sec]');
ylabel('Displacement [m]');
grid on;
figure;
plot(X_SH.Time,X_SH.Data,'Color','r','LineWidth',1.5,'DisplayName','Y2 for Skyhook');
hold on;
plot(X_pas.Time,X_pas.Data,'Color','b','LineWidth',1.5,'DisplayName','Y2 for passive');
legend('show','Location','NorthEast');
xlabel('Time [sec]');
ylabel('Angle [rad]');
grid on;


%Fa1 Fa2 - forces of the actuators
figure;
plot(Fa1.Time,Fa1.Data,'Color','r','LineWidth',1.5,'DisplayName','Fa1');
hold on;
plot(Fa2.Time,Fa2.Data,'Color','b','LineWidth',1.5,'DisplayName','Fa2');
legend('show','Location','NorthEast');
xlabel('Time [sec]');
ylabel('Force [N]');
grid on;