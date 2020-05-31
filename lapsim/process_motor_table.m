function [ motor_table_interp ] = process_motor_table( motor_table )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Summary: Interpolates the input data points of the motor RPM to torque
%         from config file

%Inputs: data points of RPM vs. torque curve for a desired motor

%[ torque, RPM ] = process_motor_table( motor_table )

%Outputs: Two column vector of the form [ RPM, torque ] for the
%         interpolated motor table

%Variables:
%motor_table: the table of RPM vs torque values
%motor_torque: a vector of the interpolated torque
%motor_RPM: a vector of the interpolated RPM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

motor_RPM = 0:1:max(motor_table(1,:)); %values from 1 to max [RPM]
motor_torque = interp1(motor_table(1,:), motor_table(2,:), motor_RPM); %interpolates motor torque for every RPM increment on provided data
motor_torque = smooth(motor_torque, 1500); %smooths data [Nm]
motor_RPM = motor_RPM';
motor_table_interp = [motor_RPM, motor_torque];

end

