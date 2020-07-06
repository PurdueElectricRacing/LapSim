classdef vd
    properties
        % Inputs
        tire_traction
        acceleration
        aero
        
        % Internal values
        suspension_params
        vehicle_geom
        
        % Outputs
        velocity
        pitch
        roll
        yaw_rate
        turn_radius
        position
        orientation
        tire_normal
    end
    
    methods
        function obj = vd(susp, vehg)
            % Vehicle dynamics class constructor
            % Input suspension parameters and vehicle geometry
            if nargin == 2
                obj.suspension_params = susp;
                obj.vehicle_geom = vehg;
            end
        end
    end
end

