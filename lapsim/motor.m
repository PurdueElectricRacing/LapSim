function [ batt_current, motor_torque ] = motor( batt_ocv, batt_current_last, pack_R, motor_table_interp, motor_RPM, batt_current_limit, power_limit, max_torque_traction, max_motor_power, motor_eff, max_motor_current)
%[ batt_current, torque ] = motor( batt_ocv, rpm, pedal, cell_s, cell_p, motor_table  )
% This function calculates the motor torque and battery current required
% for that torque.

%Variables:
%batt_ocv: battery open circuit voltage [V]
%motor_RPM: motor rotations per minute [rpm]
%pedal: pedal position [0-1]
%motor_table_interp: the torque output by the motor [Nm]
%batt_current: the battery current required to produce the torque [A]
%motor_torque: torque out of the motor [Nm]
%KV: motor rotations per volt [RPM/V] 

batt_voltage = batt_ocv - (batt_current_last * pack_R); %calculates battery voltage as open circuit voltage - current * internal resistance

motor_RPM = round(motor_RPM);

%CALCULATE MOTOR TORQUE AND MOTOR POWER

if motor_RPM > max(motor_table_interp(:, 1))
    motor_power = max_motor_power; %units in watts
    motor_torque = motor_power / (motor_RPM * 0.104719755);
else
    max_torque = motor_table_interp(motor_RPM + 1, 2);
    motor_torque = max_torque; %units in Nm
    motor_power = (motor_RPM * 0.104719755) * motor_torque; %units in watts
end

%CALCULATE BATTERY POWER AND CURRENT

batt_power = motor_power / motor_eff;
batt_current = batt_power / batt_voltage;


if motor_power > max_motor_power
    motor_power = max_motor_power;
    motor_torque = motor_power / (motor_RPM * 0.104719755);
    batt_power = motor_power / motor_eff;
    batt_current = batt_power / batt_voltage;
  
end

if motor_power > power_limit * motor_eff %aidan changes this line
    motor_power = power_limit * motor_eff; %aidan changes this line
    motor_torque = motor_power / (motor_RPM * 0.104719755);
    batt_power = motor_power / motor_eff;
    batt_current = batt_power / batt_voltage;
   
end

if batt_current > batt_current_limit
    batt_current = batt_current_limit;
    batt_power = batt_current * batt_voltage;
    motor_power = batt_power * motor_eff;
    motor_torque = motor_power / (motor_RPM * 0.104719755);
   
end

if batt_current > max_motor_current
    batt_current = max_motor_current;
    batt_power = batt_current * batt_voltage;
    motor_power = batt_power * motor_eff;
    motor_torque = motor_power / (motor_RPM * 0.104719755);
   
end
    
if motor_torque > max_torque_traction
    motor_torque = max_torque_traction;
    motor_power = motor_torque * (motor_RPM * 0.104719755);
    batt_power = motor_power / motor_eff;
    batt_current = batt_power / batt_voltage;
   
end 





%% old code using max current as input

% motor_RPM = round(motor_RPM); %rounds to whole number
% motor_voltage = motor_RPM / KV;
% 
% if motor_voltage > batt_voltage
%     motor_voltage = batt_voltage;
%     if max_motor_power / motor_voltage < max_motor_current
%         max_motor_current = max_motor_power / motor_voltage;
%     end
% end
% 
% 
% motor_current_check = 0;
% j = 1;
% 
% motor_current_check(j) = max_motor_current;
% j = j + 1;
% 
% if (motor_voltage*max_motor_current) / batt_voltage > batt_current_limit %hits current limit
%     batt_power_limit = batt_current_limit * batt_voltage;
%     motor_current_check(j) = batt_power_limit / motor_voltage;
%     j = j +1;
% end
% 
% if (motor_voltage * max_motor_current) > power_limit %hits power limit
%     motor_current_check(j) = power_limit / motor_voltage;
% end
% 
% motor_current = min(motor_current_check);
% 
% if motor_RPM > max(motor_table_interp(:,1))
%     max_torque = (max_motor_power * 9.5488) / motor_RPM;
%     max_motor_current = max_motor_power / motor_voltage;
% else
%     max_torque = motor_table_interp(motor_RPM + 1, 2); %find max torque at that RPM
% end
% 
% motor_torque = pedal * (motor_current / max_motor_current) * max_torque;
% 
% 
% if motor_torque > max_torque_traction
%     motor_torque = max_torque_traction;
%     motor_current = (motor_torque / max_torque) * max_motor_current;
% end
% 
% motor_power = motor_voltage * motor_current;
% batt_power = motor_power;
% batt_current = batt_power / batt_voltage; %calculates battery current with power / voltage

end

