function [ wheel_torque ] = driveline( motor_torque, fdr, driveline_eff)
%[ wheel_RPM, wheel_torque ] = driveline( motor_torque, motor_RPM, fdr, driveline_eff, tire_radius)
%   This function calculates the whell RPM and wheel torque.
%Variables:
%wheel_RPM: wheel speed [RPM]
%wheel_torque: wheel torque [Nm]
%motor_torque: motor_torque [Nm]
%motor_RPM: motor speed [RPM]
%fdr: final drive ratio
%driveline_eff: driveline efficiency [0-1]
%tire_radius: radius of the tire [in]

wheel_torque = (motor_torque * fdr) * (driveline_eff);



end

