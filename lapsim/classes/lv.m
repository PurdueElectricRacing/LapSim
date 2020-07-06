classdef lv
    properties
        % Inputs
        coolant_temp
        pump_rate
        batt_temp
        batt_fan_rate
        wireless_telemetry % Possibly break into own class
        mc_power
        load_budget
        
        % Internal values
        dcdc_eff
        pump_eff
        batt_fan_eff
        wt_load
        
        % Outputs
        lv_power_cons
    end
    
    methods
        function obj = lv(dce, pe, btfe)
            % Low voltage electronics class constructor
            % Input DCDC, pump, and battery fan efficiency values
            if nargin == 3
                obj.dcdc_eff = dce;
                obj.pump_eff = pe;
                obj.batt_fan_eff = btfe;
            end
        end
    end
end

