
%{
Motorcycle Setup No Acceleration

Script to initialise variables for the motorcycle model and controllers. 
The script also generates any required plots.

Conor Healy                                   12-10-2019.
%}

% Motorcycle Constants
m           = ;             % mass of the motorcycle [kg]
a           = ;             % horizontal distance from CoM to rear axle [m]
b0          = ;             % wheelbase at zero trail length [m]
h           = ;             % height of CoM (when phi, delta = 0) [m]
lambda      = ;             % front fork angle [rad]
sine_lambda = sin(lambda);  % sine of lambda []
J_s         = ;             % steering mass moment of inertia [kg m^2]
g           = 9.81;         % acceleration due to gravity [m s^-2]
r           = ;             % radius of wheels [m]

% Motorcycle Variables and Setponts
c           = ;             % trail length [m]
b           = b0 + c;       % wheelbase [m]
phi_req     = 0;            % roll angle setpoint [rad]
v_x_req     = ;             % velocity setpoint [m s^-1]

% Simplification Constants
c_0 = 1/(m*(h^2));                      % [(kg m^2)^-1]
c_1 = m*g*h;                            % [N m]
c_2 = m * a * h * sine_lambda/ b;       % [kg m]
c_3 = m * a * c * g * sine_lambda/ b;   % [N m]
c_4 = m * h * sine_lambda/ b;           % [kg]
c_5 = h * sine_lambda/ b;               % []

% Steering Motor Constants
J_m_s   = ; % steering motor moment of inertia [kg m^2]
k_m_s   = ; % steering motor machine constant [N m A^-1], [V s rad^-1]
B_m_s   = ; % steering motor viscous friction constant [N m s rad^-1]
R_a_s   = ; % steering motor armature resistance [Ohms]
L_a_s   = ; % steering motor armature inductance [H]
T_nl_s  = ; % steering motor no-load torque [N m]
T_l_s   = 0; % steering motor external load torque [N m]
n_g_s   = ; % steering motor gear ratio []
eta_g_s = 1; % steering motor gearbox efficiency []

% Drive Motor Variables
J_m_d   = ; % drive motor moment of inertia [kg m^2]
k_m_d   = ; % drive motor machine constant [N m A^-1], [V s rad^-1]
B_m_d   = ; % drive motor viscous friction constant [N m s rad^-1]
R_a_d   = ; % drive motor armature resistance [Ohms]
L_a_d   = ; % drive motor armature inductance [H]
T_nl_d  = ; % drive motor no-load torque [N m]
T_l_d   = 0; % drive motor external load torque [N m]
n_g_d   = ; % drive motor gear ratio []
eta_g_d = 1; % drive motor gearbox efficiency []

% Drive System Variables
theta   = 0;                % angle of ground plane [rad]
C_r     = ;                 % coefficient of rolling resistance [**]
rho_air = 1.2;              % density of air [kg m^-3]
C_d     = ;                 % drag coefficient [**]
A       = ;                 % front cross sectional area [m^2]
Jaxle   = ;                 % moment of inertia about rear axle [kg m^2]
A_t     = r * m * g * sin(theta);       % [N m]
B_t     = r * C_r * m * g;              % [N s]
C_t     = r * 0.5 * rho_air * C_d * A;  % [N s^2 m-1]
c_tv    = r/((r.^2) * m + Jaxle);       % [kg^-1 m^-1]

