function [ acceleration ] = acceleration_calc( wheel_torque, Cd, Af, Cr, air_density, mass, tire_radius, velocity)
%[ acceleration, velocity, disance ] = acceleration( wheel_RPM, wheel_torque, Cd, Af, Cr, air_density, mass, tire_radius)
%   This function calculates the vehicle acceleration, velocity and
%   distance
%variables:
%accleration: vehicle acceleration [m/s^2]
%velocity: vehicle velocity [m/s]
%distance: total distance traveled by the vehicle [m]
%wheel_RPM: wheel speed [RPM]
%wheel_torque :wheel torque [Nm]
%Cd: coefficient of drag
%Af: frontal area [m^2]
%Cr: rolling resistance coefficient
%air_density: air density [kg/m^3]
%mass: vehicle mass [kg]
%tire_radius: radius of the tire [in]

tire_radius = tire_radius * 0.0254; %converts to m
wheel_force = wheel_torque / tire_radius;
drag = (Cd * air_density * velocity^2 * Af) / 2;
rolling_resistance = Cr * mass * 9.8;
acceleration = (wheel_force - drag - rolling_resistance)/mass;




end

