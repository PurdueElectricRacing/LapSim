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
        cap_voltage      %voltage across the "capacitor" within the cell
        
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
        function [motor,battery,DCDC,time] = runcircuit(motor,battery,cables,DCDC,time, timestep)
            if motor.power_draw + DCDC.lv_power_cons > 0
                R = interp1(battery.cell_r1_table(:,1),battery.cell_r1_table(:,2),battery.SOC_current, 'makima');
                R(2) = interp1(battery.battery.cell_r2_table(:,1),battery.cell_r2_table(:,2),battery.SOC_current, 'makima');
                C = interp1(battery.cell_c1_table(:,1),battery.cell_c1_table(:,2),battery.SOC_current, 'makima');
                OCV = interp1(battery.OCV_table(:,1),battery.OCV_table(:,2),battery.SOC_current);
            else
                R = interp1(battery.cell_r1_table(:,1),battery.cell_r1_table(:,3),battery.SOC_current, 'makima');
                R(2) = interp1(battery.battery.cell_r2_table(:,1),battery.cell_r2_table(:,3),battery.SOC_current, 'makima');
                C = interp1(battery.cell_c1_table(:,1),battery.cell_c1_table(:,3),battery.SOC_current, 'makima');
                OCV = interp1(battery.OCV_table(:,1),battery.OCV_table(:,3),battery.SOC_current);
            end
            R(3) = cables.cable_r(1);
            R(4) = cables.cable_r(2);
            C(2) = motor.DC_link_cap;
            C(3) = DCDC.DCDC_input_cap;
            [t,voltages] = ode15s(@(t,voltages) HV_ODE(t,voltages,OCV,R,C,motor.power_draw,DCDC.lv_power_cons), [time(end),time(end) + timestep], [battery.cap_voltage,motor.DC_link_voltage,DCDC.input_cap_voltage]);
            battery.voltage_out = (OCV+((R(1)/R(3)).*voltages(:,2))+((R(1)/R(4)).*voltages(3))-voltages(:,1))/(1+(R(1)/R(3))+(R(1)/R(4)));
            battery.current_out = ((battery.voltage_out - voltages(:,2))/R(3)) + ((battery.voltage_out - voltages(:,3))/R(4));
            battery.SOC_current = battery.SOC_current - 100 * (trapz(t/3600, battery.current_out) / battery.total_cap);
            battery.cap_voltage = voltages(:,1);
            motor.DC_link_voltage = voltages(:,2);
            DCDC.input_cap_voltage = voltages(:,3);
            time = [time;t];
            %this would also be where heat output is determined. I don't
            %know how to model that
        end
    end
%     methods
%         function obj = battery(volts, mass, com)
%             % Battery class constructor
%             % Input load voltage, pack mass, and pack CoM
%             if nargin == 3
%                 obj.LoadVoltage = volts;
%                 obj.mass_batteryPack = mass;
%                 obj.centerMass_batteryPack = com;
%             end
%         end
% 
%         function computeState(self, time_step)
%             % This function calculates battery capacity, SOC, and OCV and stores them into the battery class
%             capacity_new = ((self.SOC / 100) * self.capacity) - (self.ActualCurrent * (time_step / 3600)); %new battery amp hours
%             self.SOC = (capacity_new / self.capacity) * 100; %new battery state of charge
%             self.capacity = capacity_new;
%             self.new_OCV = battery_table_interp((uint16(round(self.pack_SOC, 2) * 100) + 1), 2) * self.cell_s; %new battery open circuit voltage
%         end
%     end
end
