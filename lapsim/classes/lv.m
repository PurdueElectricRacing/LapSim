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
        
        % Outputs
        lv_power_cons;      % Power consumption in J
    end
    
    methods
        function obj = lv(dce, pe, btfe, fmd, pmd)
            % Low voltage electronics class constructor
            % Input DCDC, pump, and battery fan efficiency values
            if nargin == 5
                obj.dcdc_eff = dce;
                obj.pump_eff = pe;
                obj.batt_fan_eff = btfe;
                obj.fan_max_draw = fmd;
                obj.pump_max_draw = pmd;
            end
            obj.battery_temp_max = 60;  % Max pack temp is 60 deg C as per rules
            obj.coolant_temp_max = 50;  % I don't know what a good value for this is
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
            temp = temp / self. pump_eff;
            self.lv_power_cons = self.lv_power_cons + temp;
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
