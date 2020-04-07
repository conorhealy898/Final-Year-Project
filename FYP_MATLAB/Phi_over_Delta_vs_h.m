
% G(s) vs h

% Motorcycle Constants
m           = 4;             % mass of the motorcycle [kg]
a           = 160e-3;       % horizontal distance from CoM to rear axle [m]
b0          = 290e-3;             % wheelbase at zero trail length [m]
lambda      = 70 * pi/180;             % front fork angle [rad]
sine_lambda = sin(lambda);  % sine of lambda []
J_s         = 0.5 * 0.8 *(10e-3)^2;             % steering mass moment of inertia [kg m^2]
g           = 9.81;         % acceleration due to gravity [m s^-2]
r           = 60e-3;             % radius of wheels [m]

% Motorcycle Variables
c           = -10e-3;             % trail length [m]
b           = b0 + c;       % wheelbase [m]
v_x         = 0.9;          % forward velocity [m s^-1]
omega       = 10;           % test frequency [rad s^-1]

h_array = linspace(0.01, 0.15, 30);
magnitude = zeros(size(h_array));

for i = 1 : size(h_array, 2)
    num_1 = a * h_array(i) * sine_lambda * v_x * omega;
    num_2 = a * abs(c) * g * sine_lambda;
    num_3 = h_array(i) * sine_lambda * v_x.^2;
    
    den_1 = b * h_array(i).^2 * omega.^2;
    den_2 = - g * h_array(i) * b;
    
    num = num_1 + num_2 + num_3;
    den = den_1 + den_2;
    
    magnitude(i) = 20 * log10(abs(num/ den));
end

figure(1)
plot(h_array, magnitude, 'ro', 'LineWidth', 1.5);
grid on;
box off;
xlabel('Height of CoM (h) [m]');
ylabel('Magnitude of Transfer Function |G(s)| [dB]');

