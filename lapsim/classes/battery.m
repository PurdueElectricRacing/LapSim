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
        batt_voltage
        res_eff
        batt_power
        batt_current_limit
        
        %Outputs
        ActualCurrent
        LoadVoltage
        heat_cell
        temperature_cell
        mass_batteryPack
        centerMass_batteryPack
        capacity
        pack_SOC
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

        function computeState(self, time_step)
            % This function calculates battery capacity, SOC, and OCV and stores them into the battery class
            capacity_new = ((self.SOC / 100) * self.capacity) - (self.ActualCurrent * (time_step / 3600)); %new battery amp hours
            self.SOC = (capacity_new / self.capacity) * 100; %new battery state of charge
            self.capacity = capacity_new;
            self.new_OCV = battery_table_interp((uint16(round(self.pack_SOC, 2) * 100) + 1), 2) * self.cell_s; %new battery open circuit voltage
        end
    end
end

