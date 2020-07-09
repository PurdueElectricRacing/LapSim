classdef battery
    properties
        % Inputs
        HV_load
        LV_load
        RegenCurrent
        CoolingInputs % Possibly make this its own object
        
        % Internal values
        old_OCV
        new_OCV
        cell_s
        cell_p
        cell_internalR
        cell_OCVvsTime
        cell_r1
        cell_c1
        
        %Outputs
        ActualCurrent
        LoadVoltage
        heat_cell
        temperature_cell
        mass_batteryPack
        centerMass_batteryPack
    end
    
    methods
        function obj = battery(volts, mass, com)
            % Battery class constructor
            % Input load voltage, pack mass, and pack CoM
            if nargin == 3
                obj.LoadVoltage = volts;
                obj.mass_batteryPack = mass;
                obj.centerMass_batteryPack = com;
            end
        end
    end
end

