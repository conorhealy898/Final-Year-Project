function [K_c, zero_PL, pole_PL] = ... 
    dtime_PL(c_time_sys_OL, omega_n, zeta, T_s, caption)

d_time_sys_OL = c2d(c_time_sys_OL, T_s, 'zoh');
[pole_sys, zero_sys] = pzmap(d_time_sys_OL);

desired_point = zeta * omega_n *(-1 + tan(acos(zeta))*1i);
desired_point = exp(desired_point * T_s);
zero_PL = real(desired_point);

sys_r_vector = desired_point * ones(size(zero_sys)) - zero_sys;
sys_R_vector = desired_point * ones(size(pole_sys)) - pole_sys;

r_vector = [sys_r_vector; desired_point - zero_PL];

phase_top = 0;
phase_bottom = 0;

for x = 1 : size(r_vector, 1)
    phase_top = phase_top + angle(r_vector(x));
end

for x = 1 : size(sys_R_vector, 1)
    phase_bottom = phase_bottom + angle(sys_R_vector(x));
end

phase_pole_PL = pi + phase_top - phase_bottom;
pole_PL = real(desired_point) - imag(desired_point)/tan(phase_pole_PL);

PL = tf([1 -zero_PL], [1, -pole_PL], T_s);
sys_PL = series(PL, d_time_sys_OL);
K_c = abs(real(1/ evalfr(sys_PL, desired_point)));

figure
rlocusplot(sys_PL);
hold on;
pole = rlocus(sys_PL, K_c);
plot(real(pole),imag(pole),'mx','Markersize',10);
title(caption);
hold off;
zgrid;
end

