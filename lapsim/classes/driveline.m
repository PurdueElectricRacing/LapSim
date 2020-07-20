classdef driveline
    properties
        % Characteristics
        type                    % Hub motor vs chain drive
        configuration           % Rear wheel drive vs AWD (good for TC/TV testing)
        
        % Inputs
        motor_output_torque
        torque_load
        %state ??like turning/cornering
        
        % Internal values
        mechanical_efficiency
        gear_ratio
        moment_inertia
        dif_locking_coefficient
        chain_tension           % If applicable
        
        % Outputs
        wheel_torque_left
        wheel_torque_right
    end
    
    methods
        function obj = driveline(t, c, mech_e, gr, moi, dlc, ct)
            % Driveline class constructor
            % Input type, config, and internal values
            if nargin == 7
                obj.type = t;
                obj.configuration = c;
                obj.mechanical_efficiency = mech_e; % percentage
                obj.gear_ratio = gr;
                obj.moment_inertia = moi;
                obj.dif_locking_coefficient = dlc;
                obj.chain_tension = ct; %percentage ?
            end
        end
        
        function [wheel_torque_left,wheel_torque_right] = RunDriveline(motor_output_torque,torque_load)
            if motor_output_torque > torque_load % acceleration
                torque_load = torque_load * obj.gear_ratio;
                wheel_torque_left = obj.mechanical_efficinecy * obj.chain_tension * 0.5 * torque_load;
                wheel_torque_right = obj.mechanical_efficinecy * obj.chain_tension * 0.5 * torque_load;
            elseif motor_output_torque < torque_load % deceleration
                torque_load = motor_output_torque * obj.gear_ratio;
                wheel_torque_left = obj.mechanical_efficinecy * obj.chain_tension * 0.5 * torque_load;
                wheel_torque_right = obj.mechanical_efficinecy * obj.chain_tension * 0.5 * torque_load;
            elseif motor_output_torque == torque_load % steady state
                motor_output_torque = motor_output_torque * obj.gear_ratio;
                wheel_torque_left = obj.mechanical_efficinecy * obj.chain_tension * 0.5 * motor_output_torque;
                wheel_torque_right = obj.mechanical_efficinecy * obj.chain_tension * 0.5 * motor_output_torque;
            end
        end
        
        function [LShaft,RShaft] = differential(load_torque)
        end
    end
end

