function [K_c_d, zero_PL_d, pole_PL_d] = ... 
    MPZ_PL(K_c_a, zero_PL_a, pole_PL_a, T_s, c_time_sys_OL, caption)

zero_PL_d   = exp(zero_PL_a * T_s);
pole_PL_d   = exp(pole_PL_a * T_s);

DCG_a_PZ    = (zero_PL_a/ pole_PL_a);
DCG_d_PZ    = (1 - zero_PL_d)/(1 - pole_PL_d);

K_c_d       = K_c_a * DCG_a_PZ / DCG_d_PZ;

PL_d        = tf([1 -zero_PL_d], [1, -pole_PL_d], T_s);

d_time_sys_OL   = c2d(c_time_sys_OL, T_s, 'zoh');
sys_PL_d        = series(PL_d, d_time_sys_OL);

figure
rlocusplot(sys_PL_d);
ylim([-1.2, 1.2]);
hold on;
pole = rlocus(sys_PL_d, K_c_d);
plot(real(pole),imag(pole),'mx','Markersize',10);
title(caption);
hold off;
zgrid;
end

