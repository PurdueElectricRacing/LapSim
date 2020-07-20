classdef vd
    properties
        % Inputs
        tire_traction
        acceleration
        aero
        
        % Internal values
        suspension_params
        vehicle_geom
        tire_radius
        wheel_torque
        frontal_area
        rolling_resistance_coeff
        mass
        max_braking_force
        fdr
        Cg_z
        wheel_base
        delta_weight
        coeff_f
        max_tractive_force
        wheel_force
        
        % Outputs
        velocity
        pitch
        roll
        yaw_rate
        turn_radius
        position
        orientation
        tire_normal
        force_tractive
        lateral_g
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

        function acceleration_calc(self, aero)
            % This function calculates the vehicle acceleration, velocity and distance
            % The output (acceleration) is stored directly into the VD class
            tire_radius_conv = self.tire_radius * 0.0254; % converts to m
            wheel_force = self.wheel_torque / tire_radius_conv;
            aero.drag = (aero.Cd * aero.air_density * self.velocity ^ 2 * self.frontal_area) / 2;
            rolling_resistance = self.rolling_resistance_coeff * self.mass * 9.8;
            self.acceleration = (wheel_force - aero.drag - rolling_resistance) / self.mass;
        end

        function braking(self, driver)
            % This function calculates acceleration during braking
            self.acceleration = - (driver.brake * self.max_braking_force * 9.8);
        end

        function [braking_distance] = braking_distance_calc(self, corner_velocity)
            % This function calculates the braking distance required prior to corner entry
            braking_distance = (self.velocity ^ 2 - corner_velocity ^ 2) / (2 * self.max_braking_force * 9.8);
        end

        function [corner_velocity] = corner_speed(self, corner_radius, lateral_g, aero, tires, motor)
            % This function calculates the velocity of the car through a corner based on motor torque
            acceleration_lateral = lateral_g * 9.8;
            corner_velocity = sqrt(acceleration_lateral * corner_radius);
            aero.drag_force = (aero.cd_profile * aero.air_density * corner_velocity * aero.Af) / 2;
            rolling_resistance = aero.Cr * self.mass * 9.8;

            tire_radius_conv = self.tire_radius * 0.0254;
            self.wheel_torque = self.wheel_force * tire_radius_conv;
            motor.motor_torque = self.wheel_torque / self.fdr;
        end

        function [ max_corner_torque ] = corner_torque(self, driveline, aero, corner_count, fdr)
            r = track_table(corner_count, 3);

            self.delta_weight = self.mass * self.acceleration * self.Cg_z / self.wheel_base;
            Fc = (self.mass * self.velocity ^ 2) / r;
            Ft = (self.lateral_g * ((((self.mass / 2) + self.delta_weight) * 9.8) - (aero.cl_profile * aero.air_density * self.velocity ^ 2 * aero.Al * 0.5)));
            if Fc > Ft
                self.force_tractive = 0;
            else
                self.force_tractive = sqrt(Ft ^ 2 - Fc ^ 2);
            end
            tire_radius_conv = self.tire_radius * 0.0254;
            max_corner_torque = (tire_radius_conv * self.force_tractive * driveline.driveline_eff) / fdr;
        end

        function [ max_torque_traction ] = longitudinal_traction(self, aero, driveline, motor, fdr)
            self.delta_weight = self.mass * self.acceleration * self.Cg_z  / self.whel_base;
            self.max_tractive_force = (self.coeff_f * (((self.mass / 2) * 9.8 + self.delta_weight) - (aero.cl_profile * aero.air_density * self.velocity ^ 2 * self.Al * 0.5) * 0.5));
            tire_radius_conv = tire_radius * 0.0254;
            motor.shaft_torque = self.max_tractive_force * tire_radius_conv;
            max_torque_traction = (motor.shaft_torque * (1 / driveline.driveline_eff) / fdr);
        end
    end
end

