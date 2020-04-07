function [K_c, zero_PD] = ctime_PD(sys_OL, omega_n, zeta, caption)

desired_point = zeta * omega_n *(-1 + tan(acos(zeta))*1i);

[pole_sys, zero_sys] = pzmap(sys_OL);
sys_r_vector = desired_point * ones(size(zero_sys)) - zero_sys;
sys_R_vector = desired_point * ones(size(pole_sys)) - pole_sys;

phase_top = 0;
phase_bottom = 0;

for x = 1 : size(sys_r_vector, 1)
    phase_top = phase_top + angle(sys_r_vector(x));
end

for x = 1 : size(sys_R_vector, 1)
    phase_bottom = phase_bottom + angle(sys_R_vector(x));
end

phase_zero_PD = phase_bottom - phase_top - pi;
zero_PD = real(desired_point) - imag(desired_point)/tan(phase_zero_PD);

PD = tf([1 -zero_PD], [1]);
sys_PD = series(PD, sys_OL);
K_c = abs(real(1/ evalfr(sys_PD, desired_point)));

figure
rlocusplot(sys_PD);
hold on;
pole = rlocus(sys_PD, K_c);
plot(real(pole),imag(pole),'mx','Markersize',10);
hold off;
title(caption);
sgrid;
end
