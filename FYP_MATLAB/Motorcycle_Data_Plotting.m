
% IMU Data Plotting
clearvars;
close all;
clc;

T_s                 = 10e-3;

data                = load('Motorcycle_Test_14_2.txt');

sampleNumber        = data(:, 1);
time                = sampleNumber .* T_s;

phiReqDeg           = data(:, 2);
phiDeg              = data(:, 3);
errorRollDeg        = data(:, 4);

deltaReqDeg         = data(:, 5);
deltaDeg            = data(:, 6);
errorSteeringDeg    = data(:, 7);

v_xReq              = data(:, 8);
v_x                 = data(:, 9);
errorDrive          = data(:, 10);

VasReq              = data(:, 11);
Vas                 = data(:, 12);

VadReq              = data(:, 13);
Vad                 = data(:, 14);

figure();
subplot(5,1,1); plot(time, phiReqDeg);
hold on; plot(time, phiDeg); hold off;
grid on; box off; grid minor;
xlabel('Time [s]');
ylabel('Roll Angle [º]');
legend('Setpoint', 'Measured');
xlim([0, time(end)]);

subplot(5,1,2); plot(time, errorRollDeg);
grid on; box off; grid minor;
xlabel('Time [s]');
ylabel('Roll Error [º]');
xlim([0, time(end)]);

subplot(5,1,3); plot(time, deltaReqDeg);
hold on; plot(time, deltaDeg); hold off;
grid on; box off; grid minor;
xlabel('Time [s]');
ylabel('Steering Angle [º]');
legend('Setpoint', 'Measured');
xlim([0, time(end)]);
ylim([-5, 5]);

subplot(5,1,4); plot(time, errorSteeringDeg);
grid on; box off; grid minor;
xlabel('Time [s]');
ylabel('Steering Error [º]');
xlim([0, time(end)]);

subplot(5,1,5); plot(time, VasReq);
hold on; plot(time, Vas); hold off;
grid on; box off; grid minor;
xlabel('Time [s]');
ylabel('Steering Motor Voltage [V]');
legend('Setpoint', 'Measured');
xlim([0, time(end)]);

figure();
plot(time, phiReqDeg);
hold on; plot(time, phiDeg); hold off;
grid on; box off; grid minor;
xlabel('Time [s]');
ylabel('Roll Angle [º]');
legend('Setpoint', 'Measured');
xlim([0, time(end)]);

figure();
plot(time, deltaReqDeg);
hold on; plot(time, deltaDeg); hold off;
grid on; box off; grid minor;
xlabel('Time [s]');
ylabel('Steering Angle [º]');
legend('Setpoint', 'Measured');
xlim([0, time(end)]);
%ylim([-5, 5]);

figure();
subplot(3,1,1); plot(time, v_xReq);
hold on; plot(time, v_x); hold off;
grid on; box off; grid minor;
xlabel('Time [s]');
ylabel('Forward Velocity [m s^-^1]');
legend('Setpoint', 'Measured');
xlim([0, time(end)]);

subplot(3,1,2); plot(time, errorDrive);
grid on; box off; grid minor;
xlabel('Time [s]');
ylabel('Drive Error [m s^-^1]');
xlim([0, time(end)]);

subplot(3,1,3); plot(time, VadReq);
hold on; plot(time, Vad); hold off;
grid on; box off; grid minor;
xlabel('Time [s]');
ylabel('Drive Motor Voltage [V]');
legend('Setpoint', 'Measured');
xlim([0, time(end)]);

