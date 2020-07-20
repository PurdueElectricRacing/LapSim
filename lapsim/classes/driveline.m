classdef driveline
    properties
        % Characteristics
        type                    % Hub motor vs chain drive
        configuration           % Rear wheel drive vs AWD (good for TC/TV testing)
        
        % Inputs
        motor_output_torque
        torque_load
        
        % Internal values
        mechanical_efficiency
        gear_ratio
        moment_inertia
        dif_locking_coefficient
        chain_tension           % If applicable
        
        % Outputs
        wheel_torque
    end
    
    methods
        function obj = driveline(t, c, mech_e, gr, moi, dlc, ct)
            % Driveline class constructor
            % Input type, config, and internal values
            if nargin == 7
                obj.type = t;
                obj.configuration = c;
                obj.mechanical_efficiency = mech_e;
                obj.gear_ratio = gr;
                obj.moment_inertia = moi;
                obj.dif_locking_coefficient = dlc;
                obj.chain_tension = ct;
            end
        end
    end
end

