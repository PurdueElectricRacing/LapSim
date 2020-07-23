classdef aero
    properties
        % Inputs
        velocity
        cl_profile
        cd_profile
        cop_profile
        
        % Internal variables
        air_density
        Af
        Cr
        Al
                
        % Outputs
        drag_force
        lift_front_force
        lift_rear_force
    end
    
    methods
        function obj = aero(cL, cD, COP)
            % Aerodynamics class constructor
            % Input cL, cD, and COP profiles
            if nargin == 3
                obj.cl_profile = cL;
                obj.cd_profile = cD;
                obj.cop_profile = COP;
            end
        end
    end
end

