function [K_c, zero_PL, pole_PL] = ctime_PL(sys_OL, omega_n, zeta, caption)

desired_point = zeta * omega_n *(-1 + tan(acos(zeta))*1i);
zero_PL = real(desired_point);

[pole_sys, zero_sys] = pzmap(sys_OL);
sys_r_vector = desired_point * ones(size(zero_sys)) - zero_sys;
sys_R_vector = desired_point * ones(size(pole_sys)) - pole_sys;

phase_top = pi/2;       % phase of controller zero
phase_bottom = 0;

for x = 1 : size(sys_r_vector, 1)
    phase_top = phase_top + angle(sys_r_vector(x));
end

for x = 1 : size(sys_R_vector, 1)
    phase_bottom = phase_bottom + angle(sys_R_vector(x));
end

phase_pole_PL = pi + phase_top - phase_bottom;
pole_PL = real(desired_point) - imag(desired_point)/tan(phase_pole_PL);

PL = tf([1 -zero_PL], [1, -pole_PL]);
sys_PL = series(PL, sys_OL);
K_c = abs(real(1/ evalfr(sys_PL, desired_point)));

figure
rlocusplot(sys_PL); title(caption);
hold on;
pole = rlocus(sys_PL, K_c);
plot(real(pole),imag(pole),'mx','Markersize',10);
hold off;
sgrid;
end

