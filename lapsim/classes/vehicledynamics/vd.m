classdef vd < handle
    properties
%         % Inputs
%         tire_traction
%         acceleration
%         aero
%         
        % Internal values
        susp_params 
            %
        veh_params
            %important vehicle geometry parameters:
            %   tire_contact_xyz: location of tire contact patches (m)
            %   Cg_xyz: location of Cg relative to vehicle coords (m)
            %   mass: vehicle mass (kg)
            %   wheelbase: distance from front and rear axles (m)
            %   track_front: distance between left and right front tires (m)
            %   track_rear: see above
            %   yaw_inertia: moment of inertia in yaw (z) direction
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

        % forces acting on the vehicle at the current timestep
        % and location the forces are acting relative to vehicle coords
        % format: [x1,y1,z1; x2,y2,z2; ...]
        force_matrix = []; 
        force_location_matrix = [];

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
        
        % Previous timestep values:
        Time_p = [0 0 0];
        Pos_p = [0 0 0; 0 0 0; 0 0 0]; %current, last, last last
        Ang_pos_p = [0 0 0];
        Vel_p = [0 0 0; 0 0 0; 0 0 0];
        Ang_vel_p = [0 0 0; 0 0 0; 0 0 0];
        Accel_p = [0 0 0; 0 0 0; 0 0 0];
        Ang_accel_p = [0 0 0; 0 0 0; 0 0 0];
        
        last_update_time = 0;   %timestamp of the last time parameters were updated (s)
    end
    
    methods
        function obj = vd(susp, vehg)
            % Vehicle dynamics class constructor
            % Input suspension parameters and vehicle geometry
            if nargin == 2
                obj.susp_params = susp;
                obj.veh_params = vehg;
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
        
        function calculate_accel(self)
            %calculates vehicle accelerations using the force vector
            %calculate net force first (does not include reactions)
            net_force = sum(self.force_matrix);
            net_moment = sum(cross(self.force_location_matrix, self.force_matrix);
            
            self.Accel = net_force ./ self.veh_params.mass;
            self.Ang_accel = net_moment ./ self.veh_params.yaw_inertia;
        end
        
        function [dfr, dfl, drr, drl] = weight_transfer(self)
            %calculates the change in force on each tire due to
            %accelerations
            Cgz = self.veh_params.Cg_xyz(3);
            wheelbase = self.veh_params.wheelbase;
            track_front = self.veh_params.track_front;
            track_rear = self.veh_params.track_rear;
            mass = self.veh_params.mass;
            
            longitudinal =  self.Accel(1)*mass*Cgz/wheelbase; %positive rear
            front_lateral = self.Accel(2)*mass*Cgz/track_front; %positive right
            rear_lateral = self.Accel(2)*mass*Cgz/track_rear;
            
            dfr = -longitudinal + front_lateral;
            dfl = -longitudinal - front_lateral;
            drr = longitudinal + rear_lateral;
            drl = longitudinal - rear_lateral;
        end
               
        function update_position(self, sim_time, accel_vec, ang_accel_vec)
            %integrates accel and velocity to get position
            % sim_time is the current simulated time, used for calculating
            % the integrals
            
            time_step = sim_time - self.last_update_time;
            self.last_update_time = sim_time;
            
            %lots of math here:
            %use rungekutta.m function for integration
            
            a2 = self.Accel;
            a1 = self.Accel_p(:,1);
            a0 = self.Accel_p(:,2);
            v1 = self.Vel;
            self.Vel = rungekutta(v1, a0, a1, a2, time_step);
            
            aa2 = self.Ang_accel;
            aa1 = self.Ang_accel_p(:,1);
            aa0 = self.Ang_accel_p(:,2);
            av1 = self.Ang_vel;
            self.Ang_vel = rungekutta(av1, aa0, aa1, aa2, time_step);
            
            %not done
        end            
    end
end

