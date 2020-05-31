function [ acceleration ] = braking( max_braking_force, brake  )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

acceleration = - (brake * max_braking_force * 9.8);

end

