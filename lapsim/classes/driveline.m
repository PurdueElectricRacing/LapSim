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
        chain_tension % If applicable
        viscousCoef
        
        % Outputs
        wheel_torque_left
        wheel_torque_right
    end
    
    methods
        function obj = driveline(t, c, mech_e, gr, moi, dlc, ct, visc)
            % Driveline class constructor
            % Input type, config, and internal values
            if nargin == 8
                obj.type = t;
                obj.configuration = c;
                obj.mechanical_efficiency = mech_e; % percentage
                obj.gear_ratio = gr;
                obj.moment_inertia = moi;
                obj.dif_locking_coefficient = dlc;
                obj.chain_tension = ct; %percentage ?
                obj.viscousCoef = visc;
            end
        end
        
        function [wheel_torque_left,wheel_torque_right] = RunDriveline(motor_output_torque,torque_load)
            
            motor_output_torque = obj.chain_tension * (motor_output_torque * obj.gear_ratio);
            
            if motor_output_torque > torque_load % acceleration
                
            elseif motor_output_torque < torque_load % deceleration
                
            elseif motor_output_torque == torque_load % steady state
               
            end
            
            difTorque = differential(RAngVel,LAngVel);
            wheel_torque_left = obj.mechanical_efficinecy * (0.5 * motor_output_torque + difTorque);
            wheel_torque_right = obj.mechanical_efficinecy * (0.5 * motor_output_torque - difTorque);
            
        end
        
        function difTorque = differential(RAngVel,LAngVel)
            difTorque = (RAngVel - LAngVel) * 0.5 * obj.viscousCoef;
        end
    end
end

