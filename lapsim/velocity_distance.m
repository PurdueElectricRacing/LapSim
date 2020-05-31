function [ velocity, distance, motor_RPM ] = velocity_distance( acceleration, velocity_old, distance_old, time_step, tire_radius, fdr, track_table, corner_count )
%[ velocity, distance ] = next( acceleration, velocity_old )
%   This function calculates the next velocity and distance and motor_RPM

velocity = (acceleration * time_step) + velocity_old;

if (track_table(corner_count, 4) < velocity) && track_table(corner_count, 4) > 0
    velocity = track_table(corner_count, 4);
end

distance = (velocity_old * time_step) + distance_old;
tire_radius = tire_radius * 0.0254;
wheel_RPM = (velocity / (2*pi()*tire_radius)) *60;
motor_RPM = wheel_RPM * fdr;


end

