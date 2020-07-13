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
        eff_map             % Same here
        cable_voltage_drop  % Seems repeated given we know motor current and cabling resistance
        cable_energy_loss   % Again, same thing
        heatRej             % Also don't know here
        invPowerDraw        % Power draw of inverter in W
        
        % Outputs
        motorCurrent        % Current going to motor in A
        rpm_out             % Output rate due to command in rpm
        efficiency          % Overall HV system efficiency as a ratio (0-1)
    end
    
    methods
        function obj = hv(r, z)
            % HV class constructor
            % Input cabling resistance/impedance
            if nargin == 2
                obj.cable_r = r;
                obj.cable_z = z;
            end
        end
    end
end

