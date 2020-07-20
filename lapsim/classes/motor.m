classdef motor
    properties
        % Inputs
        armature_current
        omega_e
        rms_LtoL_voltage
        coolingPower
        motorSpeed
        
        % Internal values
        Lambda_m
        R_s
        
        % Outputs
        output_torque
        omega_rm
        output_power
        motor_eff
    end
    
    methods
        function obj = motor(lm, rs)
            % Motor class constructor
            % Input lambda_m and r_s
            if nargin == 2
                obj.Lambda_m = lm;
                obj.R_s = rs;
            end
        end
    end
end

