function [ battery_SOC, batt_ocv ] = battery( batt_current, battery_SOC_old, Ah, battery_table_interp, time_step, cell_s, batt_V_max )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Summary: Determines new battery state from drawn motor current

%Inputs: Previous battery state characteristics

%Outputs: 

%battery_SOC: new SOC [%]
%batt_ocv: new OCV [volts]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ah_new = ((battery_SOC_old/100) * Ah) - (batt_current * (time_step/3600)); %new battery amp hours
battery_SOC = (Ah_new / Ah) * 100; %new battery state of charge
batt_ocv = battery_table_interp((uint16(round(battery_SOC, 2) * 100) + 1), 2) * cell_s; %new battery open circuit voltage


end

