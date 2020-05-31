function [ corner_velocity, motor_torque ] = corner_speed( corner_radius, lateral_g, Cd, air_density, Af, Cr, mass, tire_radius, fdr )
%This function finds the maximum velocity around a corner and the
%acceleration neccessary to maintain that acceleration
%   Detailed explanation goes here

acceleration_lateral = lateral_g * 9.8;
corner_velocity = sqrt(acceleration_lateral * corner_radius);
drag = (Cd * air_density * corner_velocity^2 * Af) / 2;
rolling_resistance = Cr * mass * 9.8;
wheel_force = drag + rolling_resistance;

tire_radius = tire_radius * 0.0254;
wheel_torque = wheel_force * tire_radius;
motor_torque = wheel_torque / fdr;
end

