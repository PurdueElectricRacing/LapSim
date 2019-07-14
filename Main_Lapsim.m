clc;
clear;

% Small example simulating multiple masses across a range
% Idealy, this would be done with a variable such as max voltage, torque
% or something else more useful.

% Generate test arrays
masses = 200:300;
times = zeros(size(masses));
speeds = zeros(size(masses));

% Simulate each test case
for i = 1: numel(masses)
    mass = masses(i);
    [time, vF] = Acceleration(mass);
    times(i) = time;
    speeds(i) = vF;
end

% Dispaly results
figure(1);
subplot(1,2,1);
plot(masses, times);
grid on
title("Acceleration Simulation");
xlabel("Car Mass (Kg)");
ylabel("Acceleration Time (s)");

subplot(1,2,2);
plot(masses, speeds);
grid on
title("Acceleration Simulation");
xlabel("Car Mass (Kg)");
ylabel("Final Speed (m/s)");
