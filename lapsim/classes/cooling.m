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
        function obj = cooling(at)
            % Cooling class constructor
            % Input ambient temperature
            if nargin == 1
                obj.air_temp = at;
            end
        end
    end
end

