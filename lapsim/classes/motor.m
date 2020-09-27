classdef motor < handle
    properties
        % Inputs
        armature_current %I have no intention of using these variables but someone put them here so whatevs
        omega_e
        rms_LtoL_voltage
        coolingPower
        
        
        % Internal values
        Lambda_m
        R_s
        max_motor_power
        max_motor_current
        max_torque %cuts off power limit if torque determination is too high
        max_speed
        motor_power
        motor_voltage
        motor_current
        power_limit %this helps determine max torque at any given speed
        KV
        DC_link_cap %inverter DC link capacitance
        DC_link_voltage %inverer DC input volatage last
        motor_eff_table_discharge %efficiency map of motor when in motoring mode
        motor_eff_table_charge %efficiency map of motor when in regen mode
        motor_trq_table
        motor_speed_table
        
        % Outputs
        output_torque
        omega_rm
        output_power
        motor_eff
        motor_torque
        shaft_torque
        motor_speed
        power_draw;
    end
    
    methods
        function obj = motor(raw_vals)
            obj.Lambda_m          = raw_vals(1);
            obj.R_s               = raw_vals(2);
            obj.max_motor_power   = raw_vals(3);
            obj.max_motor_current = raw_vals(4);
            obj.max_torque        = raw_vals(5);
            obj.power_limit       = raw_vals(6);
            obj.KV                = raw_vals(7);
            obj.DC_link_cap       = raw_vals(8);
            obj.power_draw        = 0;
            
            table = readtable('Emrax_Efficiency_Data.csv');
            obj.motor_eff_table_discharge = table2array(table(2:48, 27:49));
            obj.motor_eff_table_discharge(isnan(obj.motor_eff_table_discharge)) = 60;
            obj.motor_eff_table_charge = obj.motor_eff_table_discharge;
            obj.motor_trq_table = table2array(table(2:48, 26));
            obj.motor_trq_table = obj.motor_trq_table * (30 / 230);
            obj.motor_trq_table = repmat(obj.motor_trq_table, 1, size(obj.motor_eff_table_discharge, 2));
            obj.motor_speed_table = table2array(table(1, 27:49));
            obj.motor_speed_table = obj.motor_speed_table * (11000 / 5500);
            obj.motor_speed_table = repmat(obj.motor_speed_table, size(obj.motor_eff_table_discharge, 1), 1);
            obj.max_torque = obj.motor_trq_table(end,1);
            obj.max_speed = obj.motor_speed_table(1,end);
        end

        % TODO: Input motor_table_interp
        
        function motor_run(self, bat, v, dlne)
            %bat.batt_voltage = bat.new_OCV - (bat.ActualCurrent * bat.res_eff);
            self.motor_speed = (dlne.gear_ratio * v.velocity * 60) / (2 * pi() * v.wheel_radius);
            self.motor_torque = v.wheel_torque / dlne.gear_ratio;
            if self.motor_torque < 0
                self.power_draw = -1;
                return
            end
            if self.motor_torque > self.max_torque
                disp("Over torque. This doesn't work homies.")
                disp(self.motor_torque);
            end
            mechanical_power = (self.motor_speed * self.motor_torque * 2 * pi()) / 60;
            efficiency = interp2(self.motor_speed_table, self.motor_trq_table, self.motor_eff_table_discharge, self.motor_speed, self.motor_torque);
            self.power_draw = mechanical_power * efficiency / 100;
            if self.power_draw > 60000
                disp("too much power homie")
            end
            

%             if self.motor_speed > self.max_speed
%                 self.motor_power = self.max_motor_power;
%                 self.motor_torque = self.motor_power / (self.motor_speed * 0.104719755);
%             else
%                 self.motor_torque = motor_table_interp(self.motor_speed + 1, 2);
%                 self.motor_power = (self.motor_speed * 0.104719755) * self.motor_torque;
%             end
% 
%             bat.power_out = self.motor_power / self.motor_eff;
%             bat.current_out = bat.power_out / bat.voltage_out;
% 
%             if self.motor_power > self.max_motor_power
%                 self.motor_power = self.max_motor_power;
%                 self.motor_torque = self.motor_power / (self.motor_speed * 0.104719755);
%                 bat.power_out = self.motor_power / self.motor_eff;
%                 bat.current_out = bat.power_out / bat.voltage_out;


%             if self.motor_power > self.power_limit * self.motor_eff %aidan changes this line
%                 self.motor_power = self.power_limit * self.motor_eff; %aidan changes this line
%                 self.motor_torque = self.motor_power / (self.motor_speed * 0.104719755);
%                 bat.power_out = self.motor_power / self.motor_eff;
%                 bat.current_out = bat.power_out / bat.voltage_out;
% 
%             end
% 
%             if bat.current_out > bat.batt_current_limit
%                 bat.current_out = bat.batt_current_limit;
%                 bat.power_out = bat.current_out * bat.voltage_out;
%                 self.motor_power = bat.power_out * self.motor_eff;
%                 self.motor_torque = self.motor_power / (self.motor_speed * 0.104719755);
%             end
% 
%             if bat.current_out > self.max_motor_current
%                 bat.current_out = self.max_motor_current;
%                 bat.power_out = bat.current_out * bat.voltage_out;
%                 self.motor_power = bat.power_out * self.motor_eff;
%                 self.motor_torque = self.motor_power / (self.motor_speed * 0.104719755);
% 
%             end
% 
%             if self.motor_torque > self.max_torque_traction
%                 self.motor_torque = self.max_torque_traction;
%                 self.motor_power = self.motor_torque * (self.motor_speed * 0.104719755);
%                 bat.power_out = self.motor_power / self.motor_eff;
%                 bat.current_out = bat.power_out / bat.voltage_out;
%             end 
        end

        function motor_corner(self, battery, vd, fdr, motor_table_interp)
            tire_radius_conv = vd.tire_radius * 0.0254;
            vd.wheel_torque = vd.wheel_force * tire_radius_conv;
            self.motor_torque = vd.wheel_torque / fdr;
            self.motor_speed = round(self.motor_speed);
            self.max_torque = motor_table_interp(self.motor_speed + 1, 2);
            self.motor_current = (self.motor_torque / self.max_torque) * self.max_motor_current;
            self.motor_voltage = self.motor_speed / self.KV;
            battery.batt_voltage = battery.new_OCV - (battery.ActualCurrent * battery.res_eff);
            self.motor_power = self.motor_voltage * self.motor_current;
            battery.batt_power = self.motor_power / self.motor_eff;
            battery.ActualCurrent = battery.batt_power / battery.batt_voltage;
        end

        function motor_corner_accel(self, battery, motor_table_interp, driver)
            battery.batt_voltage = battery.new_OCV - (battery.ActualCurrent * battery.res_eff);

            self.motor_speed = round(motor.motor_speed)
            if (self.motor_speed > 5999) || ((self.motor_speed / self.KV) > battery.batt_voltage)
                self.motor_torque = 0;
                self.motor_current = 0;
                self.motor_voltage = 0;
            else
                self.max_torque = motor_table_interp(self.motor_speed + 1, 2);
                self.motor_torque = pedal * self.max_torque; %finds percent of torque requested
                if self.motor_torque > self.max_corner_torque
                    self.motor_torque = self.max_corner_torque;
                    self.motor_current = (self.motor_torque / self.max_torque) * 320;
                    self.motor_voltage = self.motor_speed / self.KV;
                else
                    self.motor_current = 320;
                    self.motor_voltage = self.motor_speed / self.KV;
                end
            end
            self.motor_power = self.motor_voltage * self.motor_current;
            battery.batt_power = self.motor_power * (1 / self.motor_eff);
            battery.ActualCurrent = battery.batt_power / battery.batt_voltage;
        end
    end
end

