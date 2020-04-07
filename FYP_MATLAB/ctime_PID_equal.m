function [K_c, zero_PID] = ctime_PID_equal(sys_OL, omega_n, zeta, caption)

desired_point = zeta * omega_n *(-1 + tan(acos(zeta))*1i);

[pole_sys, zero_sys] = pzmap(sys_OL);
sys_r_vector = desired_point * ones(size(zero_sys)) - zero_sys;
sys_R_vector = desired_point * ones(size(pole_sys)) - pole_sys;

phase_top = 0;
phase_bottom = angle(desired_point);   % phase of integrator

for x = 1 : size(sys_r_vector, 1)
    phase_top = phase_top + angle(sys_r_vector(x));
end

for x = 1 : size(sys_R_vector, 1)
    phase_bottom = phase_bottom + angle(sys_R_vector(x));
end

phase_zero_PID = 0.5 *(phase_bottom - phase_top - pi);
zero_PID = real(desired_point) - imag(desired_point)/tan(phase_zero_PID);

PID = tf([1, -2 * zero_PID, zero_PID.^2], [1, 0]);
sys_PID = series(PID, sys_OL);
K_c = abs(real(1/ evalfr(sys_PID, desired_point)));

figure
rlocusplot(sys_PID);
hold on;
pole = rlocus(sys_PID, K_c);
plot(real(pole),imag(pole),'mx','Markersize',10);
title(caption);
hold off;
sgrid;
end

