classdef vd < handle
    properties
%         % Inputs
%         tire_traction
%         acceleration
%         aero
%         
%         % Internal values
%         suspension_params
%         vehicle_geom
%         
%         % Outputs
%         velocity
%         pitch
%         roll
%         yaw_rate
%         turn_radius
%         position
%         orientation
%         tire_normal
        % Global is measured relative to track coordinates (vehicle
        % starts at [0,0,0]) 
        % Local is measured relative to vehicle coordinates
        % Assume vehicle z is always parallel to track Z
        % all units [m] or [rad]
        Pos = [0;0;0];          %[X;Y;Z] position (Global)
        Ang_pos = 0;            % yaw orientation (Global)
        Vel = [0;0;0];          %[x;y;z] velocity (local)
        Ang_vel = [0;0;0];      %[roll;pitch;yaw] velocity (local)
        Accel = [0;0;0];        %[x;y;z] acceleration (local)
        Ang_accel = [0;0;0];    %[roll;pitch;yaw] acceleration (local)
        
        last_update_time = 0;   %timestamp of the last time parameters were updated (s)
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
        
        function local_vec = global2local(self, global_vec)
            %transforms from global coordinates to local coordinates
            yaw = self.Ang_pos; %absolute yaw, or heading, measured CCW from x axis
            rotation_matrix = [cos(yaw) sin(yaw); -sin(yaw) cos(yaw)];
            local_vec = rotation_matrix * global_vec;
        end
        
        function global_vec = local2global(self, local_vec)
            %transforms from local coordinates to global coordinates
            yaw = self.Ang_pos; %absolute yaw, see above
            rotation_matrix = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
            global_vec = rotation_matrix * local_vec;
        end
            
        function update_position_orientation(self, sim_time)
            %integrates accel and velocity to get position
            % sim_time is the current simulated time, used for calculating
            % the integrals
            
            time_step = sim_time - self.last_update_time;
            self.last_update_time = sim_time;
            
            %lots of math here:
            %use rungekutta.m function for integration
        end            
    end
end

