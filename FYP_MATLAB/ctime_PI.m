function [K_c, zero_PI] = ctime_PI(sys_OL, omega_n, zeta, caption)

desired_point = zeta * omega_n *(-1 + tan(acos(zeta))*1i);

[pole_sys, zero_sys] = pzmap(sys_OL);
sys_r_vector = desired_point * ones(size(zero_sys)) - zero_sys;
sys_R_vector = desired_point * ones(size(pole_sys)) - pole_sys;

phase_top = 0;
phase_bottom = angle(desired_point);   % residue phase of integrator

for x = 1 : size(sys_r_vector, 1)
    phase_top = phase_top + angle(sys_r_vector(x));
end

for x = 1 : size(sys_R_vector, 1)
    phase_bottom = phase_bottom + angle(sys_R_vector(x));
end

phase_zero_PI = phase_bottom - phase_top - pi;
zero_PI = real(desired_point) - imag(desired_point)/tan(phase_zero_PI);

PI = tf([1 -zero_PI], [1, 0]);
sys_PI = series(PI, sys_OL);
K_c = abs(real(1/ evalfr(sys_PI, desired_point)));

figure
rlocusplot(sys_PI); title('PI Root Locus');
hold on;
pole = rlocus(sys_PI, K_c);
plot(real(pole),imag(pole),'mx','Markersize',10);
hold off; title(caption);
sgrid;
end
