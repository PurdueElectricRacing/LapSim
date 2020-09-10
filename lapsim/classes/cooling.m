classdef cooling
    properties
        % Inputs
        velocity
        air_temp
        system_p_loss
        
        % Internal values
        coolant_temp
        
        % Outputs
        sys_temp
        flow_rate
    end
    
    methods
        function obj = cooling(raw_vals)
            obj.air_temp      = raw_vals(1);
            obj.system_p_loss = raw_vals(2);
            obj.coolant_temp  = raw_vals(2); % Treat this as an intial value
        end
    end
end

