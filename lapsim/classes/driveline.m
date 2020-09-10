classdef driveline
    
    properties
        % Characteristics
        type                    % Hub motor vs chain drive
        configuration           % Rear wheel drive vs AWD (good for TC/TV testing)
        
        % Inputs
        motor_output_torque     % Output torque from Motor (Nm)
        torque_load             % Amount of Torque required for driveline to be constantly ran
        R_Ang_Velocity          % Right Wheel Angular Velocity (rad/s)
        L_Ang_Velocity          % Left Wheel Angular Velocity (rads/s)
       
        
        % Internal values
        mechanical_efficiency   % Mechanical Efficiency of Driveline [0-1.0]
        gear_ratio              % Gear ratio of driveline (real numbers)
        moment_inertia          % Moment Inertia of driveline (not used rn)
        dif_locking_coefficient % Locking Coefficient (not used rn)
        chain_tension           % Chain Tension Efficiency [0-1.0]
        viscousCoef             % LSD Viscous Coefficient
        
        % Outputs
        wheel_torque_left      % Output torque at left wheel (chain drive) [Nm]
        wheel_torque_right     % Output torque at right wheel (chain drive) [Nm]
        wheel_torque           % Output torque for individual hubmotor drive [Nm]
    end
    
    methods
        function obj = driveline(raw_vals)
            obj.type                    = raw_vals(1);
            obj.configuration           = raw_vals(2);
            obj.mechanical_efficiency   = raw_vals(3); % Percentage
            obj.gear_ratio              = raw_vals(4);
            obj.moment_inertia          = raw_vals(5);
            obj.dif_locking_coefficient = raw_vals(6);
            obj.chain_tension           = raw_vals(7); % Percentage?
            obj.viscousCoef             = raw_vals(8);
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

