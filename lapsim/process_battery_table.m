function [ battery_table_interp ] = process_battery_table( battery_table )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Summary: Interpolates the input data points of the motor SOC to voltage
%         curve

%Inputs: data points of SOC vs. voltage curve for a desired battery

%[ SOC, voltage ] = process_battery_table( battery_table )

%Outputs: Two column vector of the form [ SOC, voltage ] for the
%         interpolated battery table

%Variables:
%battery_table_interp: the table of SOC vs voltage values
%voltage: a vector of the interpolated voltage
%SOC: a vector of the SOC
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This function interpolates the battery table to give voltage for every 
%SOC
%   Detailed explanation goes here
SOC = 0:0.01:100; %battery state of charge [%]
voltage = interp1(battery_table(:,2), battery_table(:, 1), SOC); %battery voltage [V]
battery_table_interp = [SOC', voltage'];
end

