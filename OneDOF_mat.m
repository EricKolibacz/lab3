%% 2 DOF system
% Comparison Skyhook and passive control
clear all;
close all;


%% Changable parameters
%1 passive, 2 PD, 3 PID, 4 Skyhook
task = 1;
%1 under damped, 2 critically, 3 over damped
damping_opt = 1;
%1 step, 2 sinusiodal, 3 road simulation
excitation_opt = 1;
%1 show bode, 0 hide bode
show_bode = 0;




%% Parameters
s = tf('s');
m_p = 0.16;
k_p = 6.32;


%% Passive system
switch(damping_opt)
    case 1
        c_p = 0.4;
    case 2
        %Critcally dumped
        c_p = 2*sqrt(k_p*m_p);
    case 3
        c_p = 2*2*sqrt(k_p*m_p);
end


zeta = c_p/(2*sqrt(k_p*m_p));
omega_n =  sqrt(k_p/m_p);


G_pas = (2*zeta*omega_n*s+omega_n^2)/(s^2 + 2*zeta*omega_n*s+omega_n^2);
%        dp = 1;
%        dd = 1;
%        omega_PD = sqrt((dp+k_p)/m_p)
%        zeta_PD = dd/m_p/2/omega_PD

%% PD and Skyhook system
dp = omega_n^2*m_p-k_p; %will be 0
dd = 2*zeta*omega_n*m_p;%will be c_p
%Task 2.3 - tuning
omega_PD = omega_n;
zeta_PD = 0.5;
dp = omega_PD^2*m_p-k_p; %will be 0
if dp == 0 && task == 2
    dp = 0.01;
end
dd = 2*zeta_PD*omega_PD*m_p;
G_PD = k_p/(m_p*s^2+dd*s+dp+k_p);


%% PID system
hp = 0.01;
hd = 1;
hi = 10;
G_PID = s*k_p/(m_p*s^3+hd*s^2+(k_p+hp)*s+hi);





%% Bode
if show_bode
    figure(1);
    bode(G_pas);%,'DisplayName','Passive control');
    grid on
    hold on;
    if task == 2 || task == 4
        bode(G_PD);%,'DisplayName','PD control');
    end
    if task == 3
        bode(G_PID);%,'DisplayName','PD control');
    end
    legend('show');

    figure(2);
end


%% Excitation
switch(excitation_opt)
    case 1
        use_step = 1;
        sim OneDOF
    case 2
        use_step = 0;
        sim OneDOF
    case 3
        
end
if excitation_opt < 3
    plot(z_p.Time,z_p.Data,'Color','r','LineWidth',1.5,'DisplayName','Output for passive');
    hold on
    plot(ref.Time,ref.Data,'Color','k','LineWidth',1.1,'DisplayName','Input');
    if task == 2
       plot(z_p_PD.Time,z_p_PD.Data,'Color','b','LineWidth',1.1,'DisplayName','Output for PD');
    end 
    if task == 4
       plot(z_p_PD.Time,z_p_PD.Data,'Color','b','LineWidth',1.1,'DisplayName','Output for Skyhook');
    end 
    if task == 3
       plot(z_p_PID.Time,z_p_PID.Data,'Color','b','LineWidth',1.1,'DisplayName','Output for PID');
    end 
    legend('show','Location','NorthEast');
    xlabel('Time [sec]');
    ylabel('Displacement [m]');
    grid on;
else
    n = 1000;
    omega = linspace(0,25,n);%logspace(-3,3,n);%linspace(0.001,100,10000);
    Sw_spatial = 4.028*10^(-7)./(2.88*10^(-4)+0.68.*omega.*omega+omega.*omega.*omega.*omega);
    Sw = 1/50*Sw_spatial;
    H = abs(((2*zeta*omega_n*1i.*omega+omega_n^2)./(1i.*omega.*1i.*omega + 2*zeta*omega_n*1i.*omega+omega_n^2)));
    H_PID = abs(1i.*omega*k_p./(m_p*(1i.*omega).^3+hd*(1i.*omega).^2+(k_p+hp)*1i.*omega+hi));
    H_PD = abs(k_p./(m_p*1i.*omega.^2+dd*1i.*omega+dp+k_p));
    Sp = H.^2.*Sw;
    Sp_PID = H_PID.^2.*Sw;
    Sp_PD = H_PID.^2.*Sw;
    loglog(omega,Sp,'DisplayName','S_p');
    hold on;
    if task == 3
        loglog(omega,Sp_PID,'DisplayName','S_p for PID');
    end;
    if task == 2
        loglog(omega,Sp_PD,'DisplayName','S_p for PD');
    end;
    if task == 4
        loglog(omega,Sp_PD,'DisplayName','S_p for Skyhook');
    end;
    xlim([0 omega(n)])
    grid on;
    legend('show','Location','NorthEast');
    xlabel('Frequency [rad/s]');
    ylabel('Energy ');
end
