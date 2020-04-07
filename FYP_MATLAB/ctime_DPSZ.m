function [K_c, zero_DPSZ1, pole_DPSZ1, pole_DPSZ2] = ... 
    ctime_DPSZ(sys_OL, omega_n, zeta, caption)

desired_point = zeta * omega_n *(-1 + tan(acos(zeta))*1i);
zero_DPSZ1 = real(desired_point);

[pole_sys, zero_sys] = pzmap(sys_OL);
sys_r_vector = desired_point * ones(size(zero_sys)) - zero_sys;
sys_R_vector = desired_point * ones(size(pole_sys)) - pole_sys;

pole_DPSZ1 = zero_sys(1);

phase_top = pi/2; 
phase_bottom = angle(desired_point - pole_DPSZ1);

for x = 1 : size(sys_r_vector, 1)
    phase_top = phase_top + angle(sys_r_vector(x));
end

for x = 1 : size(sys_R_vector, 1)
    phase_bottom = phase_bottom + angle(sys_R_vector(x));
end

phase_pole_DPSZ2 = pi + phase_top - phase_bottom;
pole_DPSZ2 = real(desired_point) - imag(desired_point)/tan(phase_pole_DPSZ2);

DPSZ = zpk([zero_DPSZ1], [pole_DPSZ1, pole_DPSZ2], 1);
sys_DPZ = series(DPSZ, sys_OL);
K_c = abs(real(1/ evalfr(sys_DPZ, desired_point)));

figure
rlocusplot(sys_DPZ); title(caption);
hold on;
pole = rlocus(sys_DPZ, K_c);
plot(real(pole),imag(pole),'mx','Markersize',10);
hold off;
sgrid;
end

