
% STEERING SYSTEM IDENTIFICATION
clearvars
clf
clc

% Motorcycle Constants
m           = 4;             % mass of the motorcycle [kg]
a           = 160e-3;       % horizontal distance from CoM to rear axle [m]
b0          = 290e-3;             % wheelbase at zero trail length [m]
h           = 90;             % height of CoM (when phi, delta = 0) [m]
lambda      = 70;             % front fork angle [rad]
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
T_l_s   = 0; % steering motor external load torque [N m]
n_g_s   = 10; % steering motor gear ratio []
eta_g_s = 0.95; % steering motor gearbox efficiency []

% Frequency Response
sine_amplitude       = 1.0;         % amplitude of sinusoidal input

N_freq = 10;
counter = 0;
omegaArray  = logspace(-1, 4, N_freq);  % array of angular frequencies
sOmegaArray = size(omegaArray);     % variable to store the array size

magnitude   = zeros(sOmegaArray);   % magnitude of output
gain  = zeros(sOmegaArray);         % gain of transfer function
dt      = zeros(sOmegaArray);
phase   = zeros(sOmegaArray);
Tsim = 100;
omega_sine      = 10;
max_time_step   = (1.5e-2)/omega_sine;
%{
for x = 1 : size(omegaArray, 2)
    
    omega_sine      = omegaArray(x);
    max_time_step   = (1.5e-2)/omega_sine;
    T               = 2 * pi/ omega_sine;   % Period of sinusoid
    Ts              = 100;                 % System settling time
    Tsim            = Ts + 3 * T;           % Simulation stop time
    nT              = Ts/T;                 % Number of periods ...
                                            % before settling
    counter = counter + 1;
    fprintf('frequency: %d of %d\n', counter, N_freq);
    
    sim('SI2_Steering_System');
   
    tmin = find(t > T * (ceil(nT) + 1/4), 1, 'first');                                              
    tmax = find(t > T * (ceil(nT) + 5/4), 1, 'first');
    
    sine_out_SS = sine_out(tmin : tmax);   % steady state output vector
    tSS = t(tmin : tmax);             % steady state time vector
    
    magnitude(x) = (max(sine_out_SS) - min(sine_out_SS)) / 2;
    gain(x) = magnitude(x) / sine_amplitude;
    gain(x) = 20 * log10(gain(x));

    [~, Imax] = max(sine_out_SS);
    dt(x) = tSS(1) - tSS(Imax);
    phase(x) = 360 * dt(x) * omega_sine/ (2 * pi);
end

figure(1)           
subplot(2, 1, 1);       % magnitude bode plot
semilogx(omegaArray, gain, 'ro', 'LineWidth', 2);
grid on;
box off;
xlabel('Frequency (\omega) [rad s^-^1]');
ylabel('Gain of Transfer Function [dB]');

subplot(2, 1, 2)        % phase bode plot  
semilogx(omegaArray, phase, 'ro', 'LineWidth', 2);
grid on;
box off;
xlabel('Frequency (\omega) [rad s^-^1]');
ylabel('Phase of Transfer Function [°]');

fprintf('Finished')
%}
