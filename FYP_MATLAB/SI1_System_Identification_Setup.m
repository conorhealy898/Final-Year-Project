
% STEERING SYSTEM IDENTIFICATION
clearvars
clc

% Motorcycle Constants
m           = 4;             % mass of the motorcycle [kg]
a           = 160e-3;       % horizontal distance from CoM to rear axle [m]
b0          = 290e-3;             % wheelbase at zero trail length [m]
h           = 90;             % height of CoM (when phi, delta = 0) [m]
lambda      = 70 * pi/180;             % front fork angle [rad]
sine_lambda = sin(lambda);  % sine of lambda []
J_s         = 0.5 * 0.8 *(10e-3)^2;             % steering mass moment of inertia [kg m^2]
g           = 9.81;         % acceleration due to gravity [m s^-2]
r           = 60e-3;             % radius of wheels [m]

% Motorcycle Variables
c           = -10e-3;             % trail length [m]
b           = b0 + c;       % wheelbase [m]
phi         = 0;             % roll angle [rad]

% Simplification Constants
c_0 = 1/(m *(h^2));                     % [(kg m^2)^-1]
c_1 = m * g * h;                        % [N m]
c_2 = m * a * h * sine_lambda/ b;       % [kg m]
c_3 = m * a * c * g * sine_lambda/ b;   % [N m]
c_4 = m * h * sine_lambda/ b;           % [kg]
c_5 = h * sine_lambda/ b;               % []

% Steering Motor Constants: Maxon ******
J_m_s   = 14.9e-7; % steering motor moment of inertia [kg m^2]
k_m_s   = 25.6e-3; % steering motor machine constant [N m A^-1], [V s rad^-1]
B_m_s   = (1e-3/31.9)*2*pi/60; % steering motor viscous friction constant [N m s rad^-1]
R_a_s   = 2.19; % steering motor armature resistance [Ohms]
L_a_s   = 278e-6; % steering motor armature inductance [H]
T_nl_s  = 27.1e-3 * k_m_s; % steering motor no-load torque [N m]
n_g_s   = 10; % steering motor gear ratio []
eta_g_s = 0.95; % steering motor gearbox efficiency []

% Drive Motor Constants: Maxon ******
J_m_d   = 14.9e-7; % drive motor moment of inertia [kg m^2]
k_m_d   = 25.6e-3; % drive motor machine constant [N m A^-1], [V s rad^-1]
B_m_d   = (1e-3/31.9)*2*pi/60;; % drive motor viscous friction constant [N m s rad^-1]
R_a_d   = 2.19; % drive motor armature resistance [Ohms]
L_a_d   = 278e-6; % drive motor armature inductance [H]
T_nl_d  = 27.1e-3 * k_m_s; % drive motor no-load torque [N m]
n_g_d   = 50; % drive motor gear ratio []
eta_g_d = 0.95; % drive motor gearbox efficiency []

% Drive System Variables
theta   = 0;                % angle of ground plane [rad]
C_r     = 0.01;                 % coefficient of rolling resistance [**]
rho_air = 1.2;              % density of air [kg m^-3]
C_d     = 0.2;                 % drag coefficient [**]
A       = 20e-4;                 % front cross sectional area [m^2]
Jaxle   = 0.5 * 200e-3 * r.^2;                 % moment of inertia about rear axle [kg m^2]
A_t     = r * m * g * sin(theta);       % [N m]
B_t     = r * C_r * m * g;              % [N s]
C_t     = r * 0.5 * rho_air * C_d * A;  % [N s^2 m-1]
c_tv    = r/((r.^2) * m + Jaxle);       % [kg^-1 m^-1]




