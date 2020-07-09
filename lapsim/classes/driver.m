classdef driver
    properties
        % Inputs
        % None, treat this as an interface
        
        % Outputs
        steering_angle
        throttle
        brake
        brake_bal
    end
    
    methods
        function obj = driver(bal)
            % Driver class constructor
            % Input initial brake balance
            if nargin == 1
                obj.brake_bal = bal;
            end
        end
    end
end

