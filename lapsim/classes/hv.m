classdef hv
    properties
        % Inputs
        torqueCommand       % Current commanded torque in N * m
        coolingPower        % Cooling power in W
        rpm_in              % Current motor rate in rpm
        
        % Internal values
        cable_r             % Effective resistance of HV cabling in ohms
        cable_z             % Effective impedance of HV cabling in ohms
        DC_link_cap         % Don't really know what we were shooting for on this one
        DCDC_input_cap      % Input capacitance of DCDC in F
        eff_map             % Same here
        cable_voltage_drop  % Seems repeated given we know motor current and cabling resistance
        cable_energy_loss   % Again, same thing
        heatRej             % Also don't know here
        invPowerDraw        % Power draw of inverter in W
        input_cap_voltage   % Voltage of DCDC input capacitance in V
        
        % Outputs
        motorCurrent        % Current going to motor in A
        rpm_out             % Output rate due to command in rpm
        efficiency          % Overall HV system efficiency as a ratio (0-1)
    end
    
    methods
        function obj = hv(raw_vals)
            obj.cable_r         = raw_vals(1);
            obj.cable_z         = raw_vals(2);
            obj.DC_link_cap     = raw_vals(3);
            obj.DCDC_input_cap  = raw_vals(4);
            input_cap_voltage   = 0;
        end
    end
end

