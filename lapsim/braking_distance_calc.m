function [ braking_distance ] = braking_distance_calc( velocity, corner_velocity, max_braking_force )
%This function calculates the braking distance given a velocity.
%   Detailed explanation goes here

    braking_distance = (velocity^2 - corner_velocity^2) / (2*max_braking_force*9.8);
end

