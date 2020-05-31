function [ batt_current, motor_torque ] = motor( batt_ocv, batt_current_last, pack_R, motor_table_interp, pedal, motor_RPM, KV, kt, motor_eff, motor_R, max_corner_torque )
%[ batt_current, torque ] = motor( batt_ocv, rpm, pedal, cell_s, cell_p, motor_table  )
% This function calculates the motor torque and battery current required
% for that torque.
%Variables
%batt_ocv: battery open circuit voltage [V]
%motor_RPM: motor rotations per minute [rpm]
%pedal: pedal position [0-1]
%cell_s: number of cells in series
%cell_p: number of cells in parrallel
%motor_table_interp: the torque output by the motor [Nm]
%batt_current: the battery current required to produce the torque [A]
%motor_torque: torque out of the motor [Nm]
%KV: motor rotations per volt [RPM/V] 
batt_voltage = batt_ocv - (batt_current_last * pack_R); %calculates battery voltage as open circuit voltage - current * internal resistance

motor_RPM = round(motor_RPM);%rounds to whole number
if (motor_RPM > 5999) || ((motor_RPM / KV) > batt_voltage);
    motor_torque = 0;
    motor_current = 0;
    motor_voltage = 0;
else
    max_torque = motor_table_interp(motor_RPM + 1, 2); %find max torque at that RPM
    motor_torque = pedal * max_torque; %finds percent of torque requested
    if motor_torque > max_corner_torque
        motor_torque = max_corner_torque;
        motor_current = (motor_torque / max_torque) * 320;
        motor_voltage = motor_RPM / KV;
    else
        motor_current = 320;
        motor_voltage = motor_RPM / KV;
    end
end
%motor_current = motor_torque / kt; %finds current required into the motor to provide requested torque
%back_emf = (motor_RPM) / KV; %finds back emf at this RPM
%motor_voltage = back_emf + (motor_current * motor_R); %calculates the voltage at the motor to provide requested current

% if back_emf > batt_voltage
%     motor_torque = 0;
%     motor_current = 0;
% elseif motor_voltage > batt_voltage
%     motor_current = (batt_voltage - back_emf) / motor_R;
%     motor_torque = motor_current * kt;
%     motor_voltage = batt_voltage;
% end
motor_power = motor_voltage * motor_current;
batt_power = motor_power * (1/motor_eff);
batt_current = batt_power / batt_voltage; %calculates battery current with power / voltage





end

