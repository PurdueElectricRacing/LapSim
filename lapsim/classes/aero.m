classdef aero < handle
    

    properties
        % Inputs
        velocity;
        cl_profile;
        cd_profile;
        cop_profile;
        
                
        % Outputs
        drag_force;
        lift_front_force;
        lift_rear_force;
    end
    
    methods
        function obj = aero(velocity) % used to be "cL, cD, COP
            cL = 3.54;
            cD = 1.03;
            roh = 1.225; % air density kg/m^3
            area = .839; % m^2
            
            % lift is 70% forward at 4.5 m/s, lift is 37% forward at 22 m/s
            COP = -.0189 * velocity + .785;
            
            % lift force and balance based on speed
            lift_force = cL * roh * .5 * area * velocity ^ 2 ;
            lift_front_force = lift_force * COP;
            lift_rear_force = lift_force * (1 - COP);
            
            % drag force based on speed
            drag_force = cD * roh * .5 * area * velocity ^ 2 ;
            
            %{
            I don't think we need this, ask aero gang if concerns
            % Aerodynamics class constructor
            % Input cL, cD, and COP profiles
            if nargin == 3
                obj.cl_profile = cL;
                obj.cd_profile = cD;
                obj.cop_profile = COP;
            end
            %}
        end
    end
end

