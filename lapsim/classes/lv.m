classdef lv < handle
    properties
        % Inputs
        coolant_temp;       % Current coolant temp in deg C
        pump_rate;          % Current pump rate as a ratio of max rate (0-1)
        batt_temp;          % Current average pack temp in deg C
        batt_fan_rate;      % Current fan rate as a ratio of max rate (0-1)
        mc_power;           % Current power draw from motor controller in W
        load_budget;        % Budget for LV load in W
        
        % Internal values
        dcdc_eff;           % Efficiency of DC/DC as a ratio (0-1)
        pump_eff;           % Efficiency of pump as a ratio (0-1)
        batt_fan_eff;       % Efficiency of battery fans as a ratio (0-1)
        fan_max_draw;       % Maximum current draw of battery fans in A
        fan_max_rate;       % Max RPM of fan
        pump_max_draw;      % Maximum current draw of pump in A
        pump_max_rate;      % Maximum pump flow rate in liters per minute
        lv_voltage;         % LV operating voltage in V
        coolant_temp_max;   % Maximum allowed coolant temperature in deg C
        battery_temp_max;   % Maximum allowed battery temperature in deg C
        fan_count;          % Number of fans in the battery
        lv_draw;            % Current draw of LV system in A
        
        % Outputs
        lv_power_cons;      % Power consumption in J
    end
    
    methods
        function obj = lv(raw_vals)
            obj.dcdc_eff         = raw_vals(1);
            obj.pump_eff         = raw_vals(2);
            obj.batt_fan_eff     = raw_vals(3);
            obj.fan_max_draw     = raw_vals(4);
            obj.fan_max_rate     = raw_vals(5);
            obj.fan_count        = raw_vals(6);
            obj.pump_max_draw    = raw_vals(7);
            obj.pump_max_rate    = raw_vals(8);
            obj.lv_voltage       = raw_vals(9);
            obj.coolant_temp_max = raw_vals(10);
            obj.lv_draw          = raw_vals(11);
            obj.battery_temp_max = 60;  % Max pack temp is 60 deg C as per rules
            obj.batt_fan_rate    = 0;
            obj.pump_rate        = 0;
        end
     
        function powerLoss(self, dt)
            % Calculates losses due to efficiency constraints
            % Inputs: dt - Time step
            temp = (-3 * 10 ^ -6) * ((self.fan_max_rate * self.batt_fan_rate) ^ 2) * self.fan_count;
            temp = temp + 0.0237 * (self.fan_max_rate * self.batt_fan_rate);
            temp = temp + 2 *10 ^ -14;
            temp = temp / self.batt_fan_eff;
            self.lv_power_cons = temp;
            temp = -0.5112 * ((self.pump_max_rate * self.pump_rate) ^ 2);
            temp = temp + 13.292 * (self.pump_max_rate * self.pump_rate);
            temp = temp - (10 ^ 13);
            temp = temp / self.pump_eff;
            if temp < 0
                temp = 0;
            end
            self.lv_power_cons = self.lv_power_cons + temp;
            self.lv_power_cons = self.lv_power_cons + (self.lv_draw * self.lv_voltage);
            self.lv_power_cons = self.lv_power_cons / self.dcdc_eff;
            self.lv_power_cons = self.lv_power_cons * dt;
        end
        
        function updateCooling(self)
            % Determines required rates for fans and pump based on temps
            if self.coolant_temp > self.coolant_temp_max                                            % Check if we are exceeding our set point
                self.pump_rate = 1;                                                                 % Turn the pump on at max rate
            end
            if self.batt_temp > (self.battery_temp_max * 0.7)                                       % Check if the battery temp is within 70% of max
                if self.batt_temp > self.battery_temp_max                                           % Check for rules violation
                   self.batt_fan_rate = 1;                                                          % Big issue, so run the fans at max rate
                else
                   self.batt_fan_rate = ((self.battery_temp_max * 0.7) / self.batt_temp) / 2 + 0.5; % Vary speed between 50 and 100% based on temp
                end
            end
        end
    end
end
