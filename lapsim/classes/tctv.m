classdef tctv
    properties
        % Inputs
        sensor_data
        driver_cmd
        specs
        
        % Internal values
        slip_ratio
        tire_angle
        
        % Outputs
        torque_command
    end
    
    methods
        function obj = tctv()
            obj.slip_ratio = 0;
            obj.tire_angle = 0;
        end
    end
end

