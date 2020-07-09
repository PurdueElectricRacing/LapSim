classdef hv
    properties
        % Inputs
        torqueCommand
        coolingPower
        rpm_in
        
        % Internal values
        cable_r
        cable_z
        DC_link_cap
        eff_map
        cable_voltage_drop % Seems repeated given we know motor current and cabling resistance
        cable_energy_loss  % Again, same thing
        heatRej
        invPowerDraw
        
        % Outputs
        motorCurrent
        rpm_out
        efficiency
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

