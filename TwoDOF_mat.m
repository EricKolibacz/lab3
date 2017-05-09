%% Task 1.1
s = tf('s');
close all;
use_step = 1;
run_time = 50;
m_p = 0.16;
%c_p = 0.4; % underdamped
%c_p = 0; % no damping
k_p = 6.32;
c_p = 0.8;
m_s = 0.16;
c_s = 0.05;
k_s = 0.0632;


%Some helping variables
alpha = s^2*m_p+(c_s+c_p)*s+(k_s+k_p);
beta = c_p*s+k_p;
gamma = c_s*s+k_s;
delta = s^2*m_s+c_s*s+k_s;

G = minreal(gamma*beta/(alpha*delta-gamma^2));
%bode(G);
%n = 1000;
%omega = linspace(0,25,n);
%s = 1i.*omega;
%grid on



%Task6.3
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