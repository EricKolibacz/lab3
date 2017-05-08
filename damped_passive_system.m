%% Task 1.1

s = tf('s');

damping_opt = 1;
excitation_opt = 2;
m_p = 0.16;
%c_p = 0.4; % underdamped
%c_p = 0; % no damping
k_p = 6.32;

switch(damping_opt)
    case 1
        c_p = 0.4;
    case 2
        c_p = 2*sqrt(k_p*m_p);
    case 3
        c_p = 2*2*sqrt(k_p*m_p);
end


zeta = c_p/(2*sqrt(k_p*m_p));
omega_n =  sqrt(k_p/m_p);


G_dis = (2*zeta*omega_n*s+omega_n^2)/(s^2 + 2*zeta*omega_n*s+omega_n^2);
figure(1);
bode(G_dis)
grid on
figure(2);

switch(excitation_opt)
    case 1
        use_step = 1;
        sim damped_sys1
    case 2
        use_step = 0;
        sim damped_sys1
    case 3
        % TODO implement track excitation
end

plot(z_p.Time,z_p.Data,'Color','r','LineWidth',1.5,'DisplayName','Output');
hold on
plot(ref.Time,ref.Data,'Color','k','LineWidth',1.1,'DisplayName','Input');
legend('show','Location','NorthEast');
xlabel('Time [sec]');
ylabel('Displacement [m]');
grid on;

