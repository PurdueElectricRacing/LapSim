classdef motor
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
        motor_power
        motor_voltage
        motor_current
        power_limit %this helps determine max torque at any given speed
        KV
        DC_link_cap %inverter DC link capacitance
        DC_link_voltage %inverer DC input volatage last
        motor_eff_table_discharge %efficiency map of motor when in motoring mode
        motor_eff_table_charge %efficiency map of motor when in regen mode
        
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
        function obj = motor(lm, rs)
            % Motor class constructor
            % Input lambda_m and r_s
            if nargin == 2
                obj.Lambda_m = lm;
                obj.R_s = rs;
            end
        end

        function motor_run(self, battery, vd, max_torque_traction)
            battery.batt_voltage = battery.new_OCV - (battery.ActualCurrent * battery.res_eff);
            self.motor_speed = round(self.motor_speed);

            if self.motor_speed > max(motor_table_interp(:,1))
                self.motor_power = self.max_motor_power;
                self.motor_torque = motor_power / (self.motor_speed * 0.104719755);
            else
                self.motor_torque = motor_table_interp(self.motor_speed + 1, 2);
                self.motor_power = (self.motor_speed * 0.104719755) * self.motor_torque;
            end

            battery.batt_power = self.motor_power / self.motor_eff;
            battery.ActualCurrent = battery.batt_power / battery.batt_voltage;

            if self.motor_power > self.max_motor_power
                self.motor_power = self.max_motor_power;
                self.motor_torque = self.motor_power / (self.motor_speed * 0.104719755);
                battery.batt_power = self.motor_power / self.motor_eff;
                battery.ActualCurrent = battery.batt_power / battery.batt_voltage;

            end

            if self.motor_power > self.power_limit * self.motor_eff %aidan changes this line
                self.motor_power = self.power_limit * self.motor_eff; %aidan changes this line
                self.motor_torque = self.motor_power / (self.motor_speed * 0.104719755);
                battery.batt_power = self.motor_power / self.motor_eff;
                battery.ActualCurrent = battery.batt_power / battery.batt_voltage;

            end

            if battery.ActualCurrent > battery.batt_current_limit
                battery.ActualCurrent = battery.batt_current_limit;
                battery.batt_power = battery.ActualCurrent * battery.batt_voltage;
                self.motor_power = battery.batt_power * self.motor_eff;
                self.motor_torque = self.motor_power / (self.motor_speed * 0.104719755);
            end

            if battery.ActualCurrent > self.max_motor_current
                battery.ActualCurrent = self.max_motor_current;
                battery.batt_power = battery.ActualCurrent * battery.batt_voltage;
                self.motor_power = battery.batt_power * self.motor_eff;
                self.motor_torque = self.motor_power / (self.motor_speed * 0.104719755);

            end

            if self.motor_torque > self.max_torque_traction
                self.motor_torque = self.max_torque_traction;
                self.motor_power = self.motor_torque * (self.motor_speed * 0.104719755);
                battery.batt_power = self.motor_power / self.motor_eff;
                battery.ActualCurrent = battery.batt_power / battery.batt_voltage;
            end 
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

