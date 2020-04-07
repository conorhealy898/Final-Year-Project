
% IMU Data Plotting

data            = load('IMU_Data.txt'); 
sampleNumber    = data(:, 1);
potAngle        = data(:, 2);
IMUAngle        = data(:, 3);

figure(); 
plot(sampleNumber, potAngle);
hold on;
plot(sampleNumber, IMUAngle);
hold off;
grid on; 
box off;
xlabel('Sample Number []');
ylabel('Angle [º]');
legend('Potentiometer', 'IMU');
