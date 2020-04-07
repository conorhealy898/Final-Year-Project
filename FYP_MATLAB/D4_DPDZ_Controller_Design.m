
% STEERING SYSTEM IDENTIFICATION
clearvars
clc
close all;

% Motorcycle Constants
m           = 1.34;          % mass of the motorcycle [kg]
b0          = 290e-3;        % wheelbase when the fork is straight [m]
c0          = -20e-3;        % Trail length when the fork is straight [m]
lambda      = 70 * pi/180;   % front fork angle [rad]
sine_lambda = sin(lambda);   % sine of lambda []
J_s         = 1e-4;          % steering mass moment of inertia [kg m^2]
g           = 9.81;          % acceleration due to gravity [m s^-2]
r           = 60e-3;         % radius of wheels [m]

a_no_stand  = 199e-3;        % horizontal distance from CoM to rear axle [m]
a_stand     = 199e-3;        % horizontal distance from CoM to rear axle [m]
h_no_stand  = 96.6e-3;       % height of CoM (when phi, delta = 0) [m]
h_stand     = 96.6e-3;       % height of CoM (when phi, delta = 0) [m]

a = a_no_stand;
h = h_no_stand;

% Motorcycle Variables

c           = -30e-3;        % trail length [m]
b           = b0 - (c - c0); % wheelbase [m]
v_x         = 0.02;           % forward velocity [m s^-1]

% Simplification Constants
c_0 = 1/(m *(h^2));                     % [(kg m^2)^-1]
c_1 = m * g * h;                        % [N m]
c_2 = m * a * h * sine_lambda/ b;       % [kg m]
c_3 = m * a * c * g * sine_lambda/ b;   % [N m]
c_4 = m * h * sine_lambda/ b;           % [kg]
c_5 = h * sine_lambda/ b;               % []

% Steering Motor Constants: Portescap Brushed DC Motor 22N78 311P
J_m_s   = 4.9e-7;   % steering motor moment of inertia [kg m^2]
k_m_s   = 15.7e-3;  % steering motor machine constant [N m A^-1], [V s rad^-1]
R_a_s   = 3.9;      % steering motor armature resistance [Ohms]
L_a_s   = 0.25e-3;  % steering motor armature inductance [H]
n_g_s   = 25;       % steering motor gear ratio []
eta_g_s = 0.8;      % steering motor gearbox efficiency []

% Drive Motor Constants: RS Pro
J_m_d   = 14.9e-7;  % drive motor moment of inertia [kg m^2]
k_m_d   = 20e-3;    % drive motor machine constant [N m A^-1], [V s rad^-1]
R_a_d   = 2.19;     % drive motor armature resistance [Ohms]
L_a_d   = 278e-6;   % drive motor armature inductance [H]
n_g_d   = 50;       % drive motor gear ratio []
eta_g_d = 0.95;     % drive motor gearbox efficiency []

% Microcontroller Variables
T_s             = 10e-3; % sampling period 
T_s_steering    = T_s;
T_s_roll        = T_s;
T_s_drive       = T_s;
V_a_max         = 11.1;
i_a_max         = 1.2;

% Drive System Variables
theta   = 10 * pi/180;          % angle of ground plane [rad]
C_r     = 0.008;                % coefficient of rolling resistance [**]
rho_air = 1.2;                  % density of air [kg m^-3]
C_d     = 0.5;                  % drag coefficient [**]
A       = 20e-4;                % front cross sectional area [m^2]
Jaxle   = 0.5 * 200e-3 * r.^2;  % moment of inertia about rear axle [kg m^2]
A_t     = r * m * g * sin(theta);       % [N m]
B_t     = r * C_r * m * g;              % [N s]
C_t     = r * 0.5 * rho_air * C_d * A;  % [N s^2 m-1]
c_tv    = r/((r.^2) * m + Jaxle);       % [kg^-1 m^-1]

% Steering System Transfer Function
C_steering_num0 = n_g_s * eta_g_s * k_m_s/ (L_a_s * J_s);
C_steering_den3 = 1;
C_steering_den2 = R_a_s/ L_a_s;
C_steering_den1 = (k_m_s.^2) * (n_g_s.^2) * eta_g_s/ (L_a_s * J_s);
C_steering_den0 = 0;
num_steering    = [C_steering_num0];
den_steering    = [C_steering_den3, C_steering_den2, C_steering_den1, ...
                   C_steering_den0];
steering_sys    = tf(num_steering, den_steering);
steering_sys_d  = c2d(steering_sys, T_s_steering);
%figure; pzmap(steering_sys); grid on;
%figure; rlocus(steering_sys); sgrid;

% Roll System Transfer Function
C_roll_num1 = a * sine_lambda * v_x/(b*h);
C_roll_num0 = a * abs(c) * g * sine_lambda/(b*h.^2) ...
              + sine_lambda * (v_x.^2)/(b*h);          
C_roll_den2 = 1;
C_roll_den1 = 0;
C_roll_den0 = -g/h;
num_roll    = [C_roll_num1, C_roll_num0];
den_roll    = [C_roll_den2, C_roll_den1, C_roll_den0];
roll_sys    = tf(num_roll, den_roll);
roll_sys_d  = c2d(roll_sys, T_s_roll);
%figure; pzmap(roll_sys); grid on;
%figure; rlocus(roll_sys); sgrid;

% Drive System Transfer Function
C_drive_num0 = n_g_d * eta_g_d * c_tv * k_m_d/ L_a_d;
C_drive_den2 = 1;
C_drive_den1 = R_a_d/ L_a_d;
C_drive_den0 = (c_tv * (k_m_d.^2) * (n_g_d.^2) ...
                * eta_g_d)/ (L_a_d * r);
num_drive    = [C_drive_num0];
den_drive    = [C_drive_den2, C_drive_den1, C_drive_den0];
drive_sys    = tf(num_drive, den_drive);
drive_sys_d  = c2d(drive_sys, T_s_drive);
%figure; pzmap(drive_sys); grid on;
%figure; rlocus(drive_sys); sgrid;

% Control Loop Requirements
omega_n_steering    = 75;
zeta_steering       = 0.707;
delta_0             = 0 * pi/180;
delta_max           = 35 * pi/180;

omega_n_roll        = 12;
zeta_roll           = 0.707;
phi_0               = 5 * pi/180;    % initial roll angle [rad]
d_phi_0             = 0 * pi/180;
d_delta_max         = inf * pi/180;
T_sim_sr            = 2;
dT_sim_sr_max       = 1e-6;
dT_sim_sr_min       = 1e-12;

omega_n_drive       = 0.1;
zeta_drive          = 0.707;
v_x_ref             = v_x;
v_x_sine            = 0;
T_sim_d_ug          = 20;
delay_v_x_ref       = 1;
delay_theta         = 10;

T_sim_motorcycle    = 4;


% Motorcycle Analogue Controllers
%

[K_PL_s, zero_PL_s, pole_PL_s] = ...
    ctime_PL(steering_sys, omega_n_steering, zeta_steering, 'Steering Phase Lead');

steering_PL     = tf([1 -zero_PL_s], [1, -pole_PL_s]);
steering_sys_PL = series(steering_PL, steering_sys);
steering_CL     = feedback(K_PL_s * steering_sys_PL, 1);
steering_and_roll_sys = series(steering_CL, roll_sys);

[K_DPDZ_r, zero_DPDZ_r1, zero_DPDZ_r2, pole_DPDZ_r1, pole_DPDZ_r2] = ... 
    ctime_DPDZ(steering_and_roll_sys, omega_n_roll, zeta_roll, -30, 'Roll DPDZ');

K_P_d = 25;

%}

% Motorcycle Analogue Control
%
sim('C4_Motorcycle_Analogue_DPDZ');

% Drive_System_Plots
figure; hold on;
%plot(time, v_x_ref_LP);
yline(v_x_ref);
plot(time, v_x); 
hold off; grid on; box off;
xlabel('t [s]');
ylabel('Forward Velocity [m s^-^1]');
legend('speed setpoint', 'speed');


figure;
subplot(3,1,1); plot(time, omega_axle_d);
hold on; plot(time, e_omega_r_d/ n_g_d); hold off;
legend('speed', 'speed error'); grid on; box off;
%title('Drive Motor Plots');
xlabel('t [s]');
ylabel('Axle Speed [rad s^-^1]')

subplot(3,1,2); plot(time, T_axle_d, 'r');
grid on; box off;
xlabel('t [s]');
ylabel('Axle Torque [N m]');

subplot(3,1,3); plot(time, T_axle_d .* omega_axle_d, 'g');
grid on; box off;
xlabel('t [s]');
ylabel('Mechanical Power [W]');



figure;
subplot(3,1,1); hold on;
plot(time, V_a_d); 
plot(time, E_a_d);
grid on; box off; hold off;
%title('Drive Motor Plots');
xlabel('t [s]');
ylabel('Voltage (V) [V]');
legend('Armature Voltage', 'Back EMF');

subplot(3,1,2); plot(time, i_a_d, 'r');
grid on; box off;
xlabel('t [s]');
ylabel('Armature Current [A]');

subplot(3,1,3); plot(time, V_a_d .* i_a_d, 'g');
grid on; box off;
xlabel('t [s]');
ylabel('Armature Power [W]');


figure;
P_a_d = V_a_d .* i_a_d;
subplot(4,1,1); plot(time, P_a_d);
grid on; box off;
%title('Drive Motor Plots');
xlabel('t [s]');
ylabel('Armature Power [W]');

P_loss_d = R_a_d .* i_a_d.^2;
subplot(4,1,2); plot(time, P_loss_d, 'r');
grid on; box off;
xlabel('t [s]');
ylabel('Armature Losses [W]')

P_mech_d = T_r_d .* omega_r_d;
subplot(4,1,3); plot(time, P_mech_d, 'g');
grid on; box off;
xlabel('t [s]');
ylabel('Motor Mechanical Power [W]');

subplot(4,1,4); plot(time, P_mech_d ./ P_a_d);
grid on; box off;
xlabel('t [s]');
ylabel('Motor Efficiency []');


% Steering and Roll Plots
figure;
subplot(3,1,1); plot(time, phi * 180/ pi); 
grid on; box off;
xlabel('t [s]');
ylabel('\phi) [°]');

subplot(3,1,2); plot(time, d_phi * 180/ pi); 
grid on; box off;
xlabel('t [s]');
ylabel('d\phi [° s^-^1]');

subplot(3,1,3); plot(time, delta * 180/ pi, 'r');
grid on; box off;
xlabel('t [s]');
ylabel('\delta [°]');


figure;
subplot(3,1,1); plot(time, omega_r_s/ n_g_s);
grid on; box off;
%title('Steering Motor Plots');
xlabel('t [s]');
ylabel('Gearbox Speed [rad s^-^1]');

subplot(3,1,2); plot(time, n_g_s * eta_g_s * T_r_s, 'r');
grid on; box off;
xlabel('t [s]');
ylabel('Gearbox Torque [N m]');

subplot(3,1,3); plot(time, T_r_s .* omega_r_s, 'g');
grid on; box off;
xlabel('t [s]');
ylabel('Mechanical Power [W]');


figure;
subplot(3,1,1); plot(time, V_a_s); 
grid on; box off;
%title('Steering Motor Plots');
xlabel('t [s]');
ylabel('Armature Voltage [V]');

subplot(3,1,2); plot(time, i_a_s, 'r');
grid on; box off;
xlabel('t [s]');
ylabel('Armature Current [A]');

subplot(3,1,3); plot(time, V_a_s .* i_a_s, 'g');
grid on; box off;
xlabel('t [s]');
ylabel('Armature Power [W]');
%}



% Motorcycle Digital Controllers
%

[K_PL_s_d, zero_PL_s_d, pole_PL_s_d] = ...
    dtime_PL(steering_sys, omega_n_steering, zeta_steering, T_s, 'Steering Phase Lead')

[K_DPDZ_r_d, zero_DPDZ_r1_d, zero_DPDZ_r2_d, pole_DPDZ_r1_d, pole_DPDZ_r2_d] = ... 
    MPZ_DPDZ(K_DPDZ_r, zero_DPDZ_r1, zero_DPDZ_r2, pole_DPDZ_r1, pole_DPDZ_r2, T_s, 'Roll DPDZ')
    
K_P_d_d = 25

%}


% Motorcycle Digital  Control
%
sim('C5_Motorcycle_Digital_DPDZ');

% Drive_System_Plots
figure; hold on;
%plot(time, v_x_ref_LP);
yline(v_x_ref);
plot(time, v_x); 
hold off; grid on; box off;
xlabel('t [s]');
ylabel('Forward Velocity [m s^-^1]');
legend('speed setpoint', 'speed');


figure;
subplot(3,1,1); plot(time, omega_axle_d);
hold on; plot(time, e_omega_r_d/ n_g_d); hold off;
legend('speed', 'speed error'); grid on; box off;
%title('Drive Motor Plots');
xlabel('t [s]');
ylabel('Axle Speed [rad s^-^1]')

subplot(3,1,2); plot(time, T_axle_d, 'r');
grid on; box off;
xlabel('t [s]');
ylabel('Axle Torque [N m]');

subplot(3,1,3); plot(time, T_axle_d .* omega_axle_d, 'g');
grid on; box off;
xlabel('t [s]');
ylabel('Mechanical Power [W]');



figure;
subplot(3,1,1); hold on;
plot(time, V_a_d); 
plot(time, E_a_d);
grid on; box off; hold off;
%title('Drive Motor Plots');
xlabel('t [s]');
ylabel('Voltage (V) [V]');
legend('Armature Voltage', 'Back EMF');

subplot(3,1,2); plot(time, i_a_d, 'r');
grid on; box off;
xlabel('t [s]');
ylabel('Armature Current [A]');

subplot(3,1,3); plot(time, V_a_d .* i_a_d, 'g');
grid on; box off;
xlabel('t [s]');
ylabel('Armature Power [W]');


figure;
P_a_d = V_a_d .* i_a_d;
subplot(4,1,1); plot(time, P_a_d);
grid on; box off;
%title('Drive Motor Plots');
xlabel('t [s]');
ylabel('Armature Power [W]');

P_loss_d = R_a_d .* i_a_d.^2;
subplot(4,1,2); plot(time, P_loss_d, 'r');
grid on; box off;
xlabel('t [s]');
ylabel('Armature Losses [W]')

P_mech_d = T_r_d .* omega_r_d;
subplot(4,1,3); plot(time, P_mech_d, 'g');
grid on; box off;
xlabel('t [s]');
ylabel('Motor Mechanical Power [W]');

subplot(4,1,4); plot(time, P_mech_d ./ P_a_d);
grid on; box off;
xlabel('t [s]');
ylabel('Motor Efficiency []');


% Steering and Roll Plots
figure;
subplot(3,1,1); plot(time, phi * 180/ pi); 
grid on; box off;
xlabel('t [s]');
ylabel('\phi) [°]');

subplot(3,1,2); plot(time, d_phi * 180/ pi); 
grid on; box off;
xlabel('t [s]');
ylabel('d\phi [° s^-^1]');

subplot(3,1,3); plot(time, delta * 180/ pi, 'r');
grid on; box off;
xlabel('t [s]');
ylabel('\delta [°]');


figure;
subplot(3,1,1); plot(time, omega_r_s/ n_g_s);
grid on; box off;
%title('Steering Motor Plots');
xlabel('t [s]');
ylabel('Gearbox Speed [rad s^-^1]');

subplot(3,1,2); plot(time, n_g_s * eta_g_s * T_r_s, 'r');
grid on; box off;
xlabel('t [s]');
ylabel('Gearbox Torque [N m]');

subplot(3,1,3); plot(time, T_r_s .* omega_r_s, 'g');
grid on; box off;
xlabel('t [s]');
ylabel('Mechanical Power [W]');


figure;
subplot(3,1,1); plot(time, V_a_s); 
grid on; box off;
%title('Steering Motor Plots');
xlabel('t [s]');
ylabel('Armature Voltage [V]');

subplot(3,1,2); plot(time, i_a_s, 'r');
grid on; box off;
xlabel('t [s]');
ylabel('Armature Current [A]');

subplot(3,1,3); plot(time, V_a_s .* i_a_s, 'g');
grid on; box off;
xlabel('t [s]');
ylabel('Armature Power [W]');
%}