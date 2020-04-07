function [K_DPDZ, zero_DPDZ1, zero_DPDZ2, pole_DPDZ1, pole_DPDZ2] = ... 
    ctime_DPDZ(sys_OL, omega_n, zeta, zero_req, caption)

desired_point = zeta * omega_n *(-1 + tan(acos(zeta))*1i);
zero_DPDZ1 = real(desired_point);
zero_DPDZ2 = zero_req;

[pole_sys, zero_sys] = pzmap(sys_OL);
sys_r_vector = desired_point * ones(size(zero_sys)) - zero_sys;
sys_R_vector = desired_point * ones(size(pole_sys)) - pole_sys;

pole_DPDZ1 = zero_sys(1);

phase_top = pi/2 + angle(desired_point - zero_DPDZ2); 
phase_bottom = angle(desired_point - pole_DPDZ1);

for x = 1 : size(sys_r_vector, 1)
    phase_top = phase_top + angle(sys_r_vector(x));
end

for x = 1 : size(sys_R_vector, 1)
    phase_bottom = phase_bottom + angle(sys_R_vector(x));
end

phase_pole_DPZ2 = pi + phase_top - phase_bottom;
pole_DPDZ2 = real(desired_point) - imag(desired_point)/tan(phase_pole_DPZ2);

DPZ = zpk([zero_DPDZ1, zero_DPDZ2], [pole_DPDZ1, pole_DPDZ2], 1);
sys_DPZ = series(DPZ, sys_OL);
K_DPDZ = abs(real(1/ evalfr(sys_DPZ, desired_point)));

figure
rlocusplot(sys_DPZ); title(caption);
hold on;
pole = rlocus(sys_DPZ, K_DPDZ);
plot(real(pole),imag(pole),'mx','Markersize',10);
hold off;
sgrid;
end

