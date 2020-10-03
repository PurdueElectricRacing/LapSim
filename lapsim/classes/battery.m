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
        function obj = battery(raw_vals)
            obj.SOC_current            = raw_vals(1);
            obj.cell_s                 = raw_vals(2);
            obj.cell_p                 = raw_vals(3);
            obj.cell_cap               = raw_vals(4);
%             obj.voltage_out            = raw_vals(5);
            
            table = readtable('VTC6_Data.csv');         % Load the cell data into Matlab
            obj.cell_r1_table = table2array(table(4:12, 15:16));
            obj.cell_r1_table = str2double(obj.cell_r1_table);
            obj.cell_r2_table = [table2array(table(4:12, 15)), table2array(table(4:12, 17))];
            obj.cell_r2_table = str2double(obj.cell_r2_table);
            obj.cell_c1_table = [table2array(table(4:12, 15)), table2array(table(4:12, 18))];
            obj.cell_c1_table = str2double(obj.cell_c1_table);
            obj.OCV_table     = table2array(table(4:1788, 6:7));
            obj.OCV_table     = str2double(obj.OCV_table);
            obj.cap_voltage   = 0.0001;
        end
        
        function total_cap = get.total_cap(obj)
            total_cap = obj.cell_s * obj.cell_p * obj.cell_cap * mean(obj.OCV_table(:,2));
        end
        
        function power_out = get.power_out(obj)
            power_out = obj.current_out .* obj.voltage_out;
            if max(abs(power_out)) > 80000  %this only does something if you ask for the max abs power
                disp('Power limit breached');
            end
        end

        function [t,x] = runcircuit(self, mo, l, h, time, timestep)
            if mo.power_draw + l.lv_power_cons > 0
                R = interp1(self.cell_r1_table(:,1),self.cell_r1_table(:,2),self.SOC_current, 'makima');
                R(2) = interp1(self.cell_r2_table(:,1),self.cell_r2_table(:,2),self.SOC_current, 'makima');
                C = interp1(self.cell_c1_table(:,1),self.cell_c1_table(:,2),self.SOC_current, 'makima');
                OCV = interp1(self.OCV_table(:,1),self.OCV_table(:,2),self.SOC_current) * self.cell_s;
            else
%                 R = interp1(self.cell_r1_table(:,1),self.cell_r1_table(:,3),self.SOC_current, 'makima');
%                 R(2) = interp1(self.battery.cell_r2_table(:,1),self.cell_r2_table(:,3),self.SOC_current, 'makima');
%                 C = interp1(self.cell_c1_table(:,1),self.cell_c1_table(:,3),self.SOC_current, 'makima');
%                 OCV = interp1(self.OCV_table(:,1),self.OCV_table(:,3),self.SOC_current);
            end
            R(3) = h.cable_r;
            R(4) = h.cable_r;
            C(2) = mo.DC_link_cap;
            C(3) = h.DCDC_input_cap;
            [t,x] = ode15s(@(t,x) HV_ODE(t,x,OCV,R,C,mo.power_draw, 0), [0, 0.1], [self.cap_voltage, mo.DC_link_voltage]); % , h.input_cap_voltage
            self.voltage_out = (OCV+((R(1)/R(3)).*x(:,2)) - x(:,1))/(1+(R(1)/R(3))); % +((R(1)/R(4)).*x(:,3))    +(R(1)/R(4))
            self.current_out = ((self.voltage_out - x(:,2))/R(3)); % + ((self.voltage_out - x(:,3))/R(4));
            self.SOC_current = self.SOC_current - 100 * (trapz(t/3600, self.power_out) / self.total_cap);
            self.cap_voltage = x(end,1);
            mo.DC_link_voltage = x(end,2);
            %h.input_cap_voltage = x(end,3);
            %this would also be where heat output is determined. I don't
            %know how to model that
        end
    end
end