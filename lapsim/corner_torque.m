function [ max_corner_torque ] = corner_torque( velocity, lateral_g, mass, track_table, corner_count, tire_radius, fdr, driveline_eff, Cl, air_density, Al, acceleration, Cg_z, wheel_base )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

r = track_table(corner_count, 3);

delta_weight = mass * acceleration * Cg_z / wheel_base; %calculates weight transfer to rear axle
Fc = (mass * velocity^2) / r;
Ft = (lateral_g * ((((mass / 2) + delta_weight) * 9.8) - (Cl * air_density * velocity^2 * Al * 0.5)));
if Fc > Ft
    F_tractive = 0;
else
    F_tractive = sqrt(Ft^2 - Fc^2);
end
tire_radius = tire_radius * 0.0254; %converts to m
max_corner_torque = (tire_radius * F_tractive * driveline_eff) / (fdr); %divided by two for rear wheel drive


end

