%% Task 1.1
clear all;
s = tf('s');
close all;
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

exictation = 0;
f = 1.1792;
run_time = 20;
A = [0 1 0 0;
    -2*k/m -2*c/m 0 0;
    0 0 0 1;
    %0 0 -2*L/J -2*L/J];
    0 0 ];
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
plot(z_SH.Time,z_SH.Data,'Color','r','LineWidth',1.5,'DisplayName','Y1 for passive');
legend('show','Location','NorthEast');
xlabel('Time [sec]');
ylabel('Displacement [m]');
grid on;
figure;
plot(X_SH.Time,X_SH.Data,'Color','b','LineWidth',1.5,'DisplayName','Y2 for passive');
legend('show','Location','NorthEast');
xlabel('Time [sec]');
ylabel('Angle [rad]');
grid on;
%figure;
%bode(G_ss(1,1))
grid on;