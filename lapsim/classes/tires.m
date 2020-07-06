classdef tires
    properties
        % Inputs
        normal_force
        steering_angle
        camber_angle
        velocity_lat
        velocity_long
        
        % Internal values
        slip_ratio
        coeff_friction
        
        % Outputs
        traction_lat
        traction_long
        torque_load
    end
    
    methods
        function obj = tires(coeff)
            % Tire class constructor
            % Input coefficient of friction
            if nargin == 1
                obj.coeff_friction = coeff;
            end
        end
    end
end

