%% 2 DOF system
% Comparison Skyhook and passive control

close all;
clear all;


%% parameters
s = tf('s');
use_step = 0;
run_time = 50;
m_p = 0.16;
%c_p = 0.4; % underdamped
%c_p = 0; % no damping
k_p = 6.32;
c_p = 0.8;
m_s = 0.16;
c_s = 0.05;
k_s = 0.0632;


%% Passive system
%Some helping variables
alpha = s^2*m_p+(c_s+c_p)*s+(k_s+k_p);
beta = c_p*s+k_p;
gamma = c_s*s+k_s;
delta = s^2*m_s+c_s*s+k_s;

%Resulting transfer function
G_2DOF = minreal(gamma*beta/(alpha*delta-gamma^2));

%Bode diagram of the system
if 0 
    bode(G_2DOF);
    hold on;
    m_p1 = 0.16;
    k_p1 = 6.32;
    c_p1 = 0.4;
    zeta = c_p1/(2*sqrt(k_p1*m_p1));
    omega_n =  sqrt(k_p1/m_p1);


    G_1DOF = (2*zeta*omega_n*s+omega_n^2)/(s^2 + 2*zeta*omega_n*s+omega_n^2);
    bode(G_1DOF);
    grid on;
    legend('show');
end



%% Skyhook system
A = [0 1 0 0;
    -k_s/m_s 0 k_s/m_s 0;
    0 0 0 1;
    k_s/m_p 0 (-k_s-k_p)/m_p -c_p/m_p];
B = [0 0 0;
    1/m_s 0 0;
    0 0 0;
    -1/m_p k_p/m_p c_p/m_p];
C = [1 0 0 0];
D = [0 0 0];

G_ss = ss(A,B,C,D);


%% Plots
if 1
    figure(1)
    T = 0.08;
    sim TwoDOF
    plot(ref.Time,ref.Data,'Color','k','LineWidth',1.1,'DisplayName','Input');
    hold on
    plot(z_SH.Time,z_SH.Data,'Color','r','LineWidth',1.5,'DisplayName','Output for Skyhook');
    plot(z_s.Time,z_s.Data,'Color','b','LineWidth',1.5,'DisplayName','Passive control');
    legend('show','Location','NorthEast');
    xlabel('Time [sec]');
    ylabel('Displacement [m]');
    grid on;
end