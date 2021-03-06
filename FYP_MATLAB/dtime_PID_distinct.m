function [K_c, zero_PID_1, zero_PID_2] = ...
          dtime_PID_distinct(c_time_sys_OL, omega_n, zeta, T_s)

d_time_sys_OL = c2d(c_time_sys_OL, T_s, 'zoh');
[pole_sys, zero_sys] = pzmap(d_time_sys_OL);

desired_point = zeta * omega_n *(-1 + tan(acos(zeta))*1i);
desired_point = exp(desired_point * T_s);
zero_PID_1 = real(desired_point);

sys_r_vector = desired_point * ones(size(zero_sys)) - zero_sys;
sys_R_vector = desired_point * ones(size(pole_sys)) - pole_sys;

phase_top = pi/2;                       % phase of zero_PID_1
phase_bottom = angle(desired_point - 1);    % phase of integrator

for x = 1 : size(sys_r_vector, 1)
    phase_top = phase_top + angle(sys_r_vector(x));
end

for x = 1 : size(sys_R_vector, 1)
    phase_bottom = phase_bottom + angle(sys_R_vector(x));
end

phase_zero_PID_2 = phase_bottom - phase_top - pi;
zero_PID_2 = real(desired_point) - ...
             imag(desired_point)/tan(phase_zero_PID_2);

PID = tf([1, -(zero_PID_1 + zero_PID_2), zero_PID_1 * zero_PID_2], ...
    [1, -1], T_s);
sys_PID = series(PID, d_time_sys_OL);
K_c = abs(real(1/ evalfr(sys_PID, desired_point)));

figure
rlocusplot(sys_PID);
hold on;
pole = rlocus(sys_PID, K_c);
plot(real(pole),imag(pole),'mx','Markersize',10);
title('Distinct PID Root Locus');
hold off;
zgrid;
end