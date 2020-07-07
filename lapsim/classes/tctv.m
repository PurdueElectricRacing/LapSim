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
        function obj = tctv(sense, specs)
            % TC/TV class constructor
            % Input sensor classand specs
            if nargin == 2
                obj.sensor_data = sense;
                obj.specs = specs;
            end
        end
    end
end

