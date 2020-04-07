function [K_DPDZ_d, zero_DPDZ1_d, zero_DPDZ2_d, pole_DPDZ1_d, pole_DPDZ2_d] = ... 
    MPZ_DPDZ(K_DPDZ_a, zero_DPDZ1_a, zero_DPDZ2_a, pole_DPDZ1_a, pole_DPDZ2_a, T_s, caption)

zero_DPDZ1_d = exp(zero_DPDZ1_a * T_s);
zero_DPDZ2_d = exp(zero_DPDZ2_a * T_s);
pole_DPDZ1_d = exp(pole_DPDZ1_a * T_s);
pole_DPDZ2_d = exp(pole_DPDZ2_a * T_s);

DCG_a_PZ = (zero_DPDZ1_a * zero_DPDZ2_a) / ((pole_DPDZ1_a)*(pole_DPDZ2_a));
DCG_d_PZ = (1 - zero_DPDZ1_d) * (1 - zero_DPDZ2_d) /((1 - pole_DPDZ1_d) * (1 - pole_DPDZ1_d));

K_DPDZ_d = K_DPDZ_a * DCG_a_PZ / DCG_d_PZ;
end

