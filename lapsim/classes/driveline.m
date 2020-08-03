classdef driveline
    
    properties
        % Characteristics
        type                    % Hub motor vs chain drive
        configuration           % Rear wheel drive vs AWD (good for TC/TV testing)
        
        % Inputs
        motor_output_torque     % Output torque from Motor (N/m)
        torque_load             % Amount of Torque required for driveline to be constantly ran
        R_Ang_Velocity          % Right Wheel Angular Velocity (rad/s)
        L_Ang_Velocity          % Left Wheel Angular Velocity (rads/s)
       
        
        % Internal values
        mechanical_efficiency   % Mechanical Efficiency of Driveline [0-1.0]
        gear_ratio              % Gear ratio of driveline (real numbers)
        moment_inertia
        dif_locking_coefficient
        chain_tension % If applicable
        viscousCoef
        
        % Outputs
        wheel_torque_left
        wheel_torque_right
        wheel_torque
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
        
        % Driveline for Rear Wheel Drive Vehicle
        
        function [wheel_torque_left,wheel_torque_right] = RWD_Driveline(self)
            
            self.motor_output_torque = obj.chain_tension * (self.motor_output_torque * obj.gear_ratio);
            
            % complex model of chain drive?
            
            %if self.motor_output_torque > self.torque_load % acceleration
                
            %elseif self.motor_output_torque < self.torque_load % deceleration
                
            %elseif self.motor_output_torque == self.torque_load % steady state
               % do nothing
            %end
            
            difTorque = differential(self.R_Ang_Velocity,self.L_Ang_Velocity);
            wheel_torque_left = obj.mechanical_efficinecy * (0.5 * self.motor_output_torque + difTorque);
            wheel_torque_right = obj.mechanical_efficinecy * (0.5 * self.motor_output_torque - difTorque);
            
        end
        
        % Basic Model of Viscous Differential
        
        function difTorque = differential(RAV,LAV)
            difTorque = (RAV - LAV) * 0.5 * obj.viscousCoef;
        end
        
        % Driveline for Individual Hub Motor in HM Vehicle
        
        function wheel_torque = HM_Driveline(self)
            wheel_torque = obj.mechanical_efficiency * (self.motor_output_torque * obj.gear_ratio);
        end
    end
end

