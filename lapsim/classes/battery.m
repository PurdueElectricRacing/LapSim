classdef battery < handle  % must include "handle" in order to pass and return 
                        % the class to functions.  Read about value vs
                        % handle classes: https://www.mathworks.com/help/matlab/matlab_oop/comparing-handle-and-value-classes.html
    properties
        % Inputs
        coolingInputs % Possibly make this its own object
        
        % Internal values
        OCV_table      %OCV vs. SOC table for charging / discharging (Volts)
        SOC_current      %current SOC (%)
        cell_s           %cells in series
        cell_p           %cells in parallel
        cell_r1_table    %r1 values vs SOC for charging / discharging (mohm)
        cell_r2_table    %r2 values vs SOC for charging / discharging (mohm)
        cell_c1_table    %c1 values vs SOC for charging / discharging (Farads)
        cell_cap         %cell capacity (Wh)
        
        %Outputs
        current_out      %Output current from the pack
        voltage_out      %Output voltage of the pack
        heat_cell        %mechanical people should better define these
        temperature_cell
        mass_batteryPack
        centerMass_batteryPack
    end
    properties (Dependent)
        total_cap
        power_out
    end
    
    methods
        function obj = battery(SOC, s, p, cap, mass, com)
            % Battery class constructor
            % Input load voltage, pack mass, and pack CoM
            if nargin == 6
                obj.SOC_current = SOC;
                obj.cell_s = s;
                obj.cell_p = p;
                obj.cell_cap = cap;
                obj.mass_batteryPack = mass;
                obj.centerMass_batteryPack = com;
            end
        end
        
        function total_cap = get.total_cap(obj)
            total_cap = obj.cell_s * obj.cell_p * obj.cell_cap;
        end
        
        function power_out = get.power_out(obj)
            power_out = obj.current_out .* obj.voltage_out;
            if max(abs(power_out)) > 80000  %this only does something if you ask for the max abs power
                disp('Power limit breached');
            end
        end
    end
end

