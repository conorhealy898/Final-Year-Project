function [K_c, zero_PI] = dtime_PI(c_time_sys_OL, omega_n, zeta, T_s)

d_time_sys_OL = c2d(c_time_sys_OL, T_s, 'zoh');
[pole_sys, zero_sys] = pzmap(d_time_sys_OL);

desired_point = zeta * omega_n *(-1 + tan(acos(zeta))*1i);
desired_point = exp(desired_point * T_s);

sys_r_vector = desired_point * ones(size(zero_sys)) - zero_sys;
sys_R_vector = desired_point * ones(size(pole_sys)) - pole_sys;

phase_top = 0;
phase_bottom = angle(desired_point - 1);   % residue phase of integrator

for x = 1 : size(sys_r_vector, 1)
    phase_top = phase_top + angle(sys_r_vector(x));
end

for x = 1 : size(sys_R_vector, 1)
    phase_bottom = phase_bottom + angle(sys_R_vector(x));
end

phase_zero_PI = phase_bottom - phase_top - pi;
zero_PI = real(desired_point) - imag(desired_point)/tan(phase_zero_PI);

PI = tf([1 -zero_PI], [1, -1], T_s);
sys_PI = series(PI, d_time_sys_OL);
K_c = abs(real(1/ evalfr(sys_PI, desired_point)));

figure
rlocusplot(sys_PI);
hold on;
pole = rlocus(sys_PI, K_c);
plot(real(pole),imag(pole),'mx','Markersize',10);
title('PI Root Locus');
hold off;
zgrid;
end
