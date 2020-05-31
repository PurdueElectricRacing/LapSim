function [ wheel_torque, motor_torque, batt_current ] = motor_corner( wheel_force, motor_RPM, batt_ocv, batt_current_last, pack_R, KV, tire_radius, fdr, max_motor_current, motor_table_interp, motor_eff )
%This function finds the battery current and motor torque to provide a
%given wheel force
%   Detailed explanation goes here

tire_radius = tire_radius * 0.0254; %converts to m
wheel_torque = wheel_force * tire_radius;
motor_torque = wheel_torque / fdr;
motor_RPM = round(motor_RPM);
max_torque = motor_table_interp(motor_RPM + 1, 2);
motor_current = (motor_torque / max_torque) * max_motor_current;
motor_voltage = motor_RPM / KV;
batt_voltage = batt_ocv - (batt_current_last * pack_R); %calculates battery voltage as open circuit voltage - current * internal resistance
motor_power = motor_voltage * motor_current;
batt_power = motor_power / motor_eff;
batt_current = batt_power / batt_voltage; 


end

