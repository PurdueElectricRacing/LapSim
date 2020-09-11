classdef vd < handle
    properties
        %Inputs
        vehicle_mass
        wheel_radius
        Cg_xyz
        wheelbase
        track_front
        track_rear
        acceleration
        tire_coef
        num_motor
        % Forces acting on the vehicle at the current timestep
        % and location the forces are acting relative to vehicle coords
        % Format: [x1,y1,z1; x2,y2,z2; ...]
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

        velocity % NOTE: THIS IS ONLY HERE UNTIL WE MOVE TO A 3 AXIS SYSTEM
        max_velocity
        
        % Previous timestep values:
        Time_p = [0 0 0];
        Pos_p = [0 0 0; 0 0 0; 0 0 0]; %current, last, last last
        Ang_pos_p = [0 0 0];
        Vel_p = [0 0 0; 0 0 0; 0 0 0];
        Ang_vel_p = [0 0 0; 0 0 0; 0 0 0];
        Accel_p = [0 0 0; 0 0 0; 0 0 0];
        Ang_accel_p = [0 0 0; 0 0 0; 0 0 0];
        k
        
        last_update_time = 0;   %timestamp of the last time parameters were updated (s)
    end
    
    methods
        function obj = vd(raw_vals)
            % Vehicle dynamics class constructor
            % Input suspension parameters and vehicle geometry
            obj.k = 1;
            obj.vehicle_mass = raw_vals(1);
            obj.wheel_radius = raw_vals(2);
            obj.Cg_xyz       = raw_vals(3:5);
            obj.wheelbase    = raw_vals(6);
            obj.track_front  = raw_vals(7);
            obj.track_rear   = raw_vals(8);
            obj.tire_coef    = raw_vals(9);
            obj.num_motor    = raw_vals(10);
            obj.velocity     = 0;
            obj.max_velocity = 0;
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
        
        function [dfr, dfl, drr, drl] = weight_transfer(self)
            %calculates the change in force on each tire due to
            %accelerations
            Cgz = self.Cg_xyz(3);
            
            longitudinal =  self.Accel(1) * mass * Cgz / self.wheelbase; %positive rear
            front_lateral = self.Accel(2) * mass * Cgz / self.track_front; %positive right
            rear_lateral = self.Accel(2) * mass * Cgz / self.track_rear;
            
            dfr = -longitudinal + front_lateral;
            dfl = -longitudinal - front_lateral;
            drr = longitudinal + rear_lateral;
            drl = longitudinal - rear_lateral;
        end
               
        function update_position(self, sim_time)
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
        
        function [fr, fl, rr, rl] = weight_transfer_variable(self, rho)
            %calculates the change in force on each tire due to
            %accelerations
            Cgz = self.Cg_xyz(3);
            
            syms v_max;
            lat_accel_capacity = v_max^2 / rho;
            
            longitudinal =  0; %self.Accel(1)*mass*Cgz/wheelbase; positive rear
            front_lateral = lat_accel_capacity*self.vehicle_mass*Cgz/self.track_front; %positive right
            rear_lateral = lat_accel_capacity*self.vehicle_mass*Cgz/self.track_rear;
            
            fr = -longitudinal + front_lateral + (.25 * self.vehicle_mass);
            fl = -longitudinal - front_lateral + (.25 * self.vehicle_mass);
            rr = longitudinal + rear_lateral + (.25 * self.vehicle_mass);
            rl = longitudinal - rear_lateral + (.25 * self.vehicle_mass);
        end
        
        function [max_speed_table] = max_corner_speed(self, track_table)
            %determines maximum corner speed for each corner and outputs to
            %table based on track input table
            
            max_speed_table = zeros(size(track_table, 1));                           % Pre-allocate size of table

            for i=1:size(track_table, 1)
                if cell2mat(track_table(i,3)) ~= 0                                            % True if line in track table is a corner not a straight
                    rho = track_table(i,3);                                         % Corner radius for current corner
                    [fr, fl, rr, rl] = weight_transfer_variable(self, rho);         % Determine weight transfer in terms of v_max
                    max_speed_table(i) = sqrt(rho * (self.tire_coef * (fr + fl + rr + rl)) / self.vehicle_mass);
                else                                                                % If line of track table is a straight
                    max_speed_table(i) = 0;                                         % Placeholder for v_max on straightaway
                end
            end
        end

        % TODO: Make sure first element of the track is not a corner
        %       Dynamic calculation of cornering speed
        %       Dynamic calculation of cornering torque
        %       Smooth corner exit for high time steps
        %       Remove '=' from last_element__ set if logic checks out
        %       Move to a system that asks motor class if we can output set
        %       torque and back off if we can't
        %       Remove calculation of rr prior to velocity update somehow
        
        % NOTE: Persistent variables will end in '__' to ensure that we
        %       don't accidentally have two variables with the same name
        function vd_main(self, ae, dr, mo, track_table, max_corner_table, dt)
            persistent element_remain__;                                            % Distance to end of element [m]
            persistent in_element__;                                                % Boolean marking if we are in a corner
            persistent last_element__;                                              % Boolean marking if we are in the last element of the track
            
            if isempty(element_remain__)                                            % Check if this is the first iteration
                element_remain__ = str2double(track_table(self.k, 2));              % It is, so load the first element
                in_element__ = true;                                                % Mark us as actively being in a new element
                last_element__ = false;                                             % Initialize to false
            end
            
            % Check if we need to move to the next element
            if in_element__ == false
                self.k = self.k + 1;                                                % Jump to the next track element
                element_remain__ = str2double(track_table(self.k, 2));              % Update the length until the exit of this element
                in_element__ = true;                                                % Mark us as actively being in a new element
                if self.k >= size(track_table, 1)                                   % Check if we are in the last element
                    last_element__ = true;                                          % Mark us as being in the last element
                end
            end
            
            % Calculate force due to rolling resistance
            rr_force = (0.005 + (1 / 1.379) * (0.01 + 0.0095 * ((self.velocity / 100) ^ 2))) * self.vehicle_mass * 9.81;
            
            % Update velocity and distance to end of element
            if str2double(track_table(self.k, 3)) ~= 0                              % Check if we are in a corner
                self.velocity = max_corner_table(self.k);                           % We are, so assume we're going max speed
                element_remain__ = element_remain__ - (dt * self.velocity);         % Update distance remaining after this iteration
                if element_remain__ <= 0                                            % Check if we have reached the end of the corner
                    in_element__ = false;                                           % We are, so move to the next track element
                end
                % Calculate and update torque used
                mo.motor_torque = (rr_force + ae.drag_force) / self.num_motor * self.wheel_radius;
            else                                                                    % We aren't in a corner
                % Calculate the maximum entry speed for the next corner
                corner_entry_v = sqrt(self.velocity ^ 2 - 19.62 * element_remain__);
                if length(corner_entry_v) ~= 1                                      % Check if we get an imaginary value
                    corner_entry_v = inf;                                           % Override to inf so we have a scalar again
                elseif corner_entry_v ~= real(corner_entry_v)                       % One more check
                    corner_entry_v = inf;                                           % Override to inf so we have a scalar again
                end
                if self.velocity >= corner_entry_v && ~last_element__               % Check if we need to brake
                    self.velocity = self.velocity - (19.62 * dt);                   % We do, so start slowing down
                    mo.motor_torque = 0;                                            % Command 0 torque under braking
                else                                                                % MORE POWER!!!!!!!
                    motor_force = dr.max_motor_torque / self.wheel_radius;          % Command max torque and calculate force
                    fric_force = self.tire_coef * self.vehicle_mass / 4;            % Calculate force due to friction
                    if motor_force <= fric_force                                    % Check if we've asked for too much power
                        accel_force = motor_force * self.num_motor;                 % We haven't, so calculate acceleration force
                        mo.motor_torque = dr.max_motor_torque;                      % Command max torque
                    else                                                            % We've asked for too much power
                        accel_force = fric_force * self.num_motor;                  % So scale back to max that friction will allow
                        mo.motor_torque = fric_force / self.wheel_radius;           % Update the torque used to reflect change
                    end
                    % Update velocity
                    self.velocity = self.velocity + ((((accel_force * self.num_motor) - rr_force - ae.drag_force) / self.vehicle_mass) * dt);
                end
                element_remain__ = element_remain__ - (dt * self.velocity);         % Update distance remaining after this iteration
                if element_remain__ <= 0                                            % Check if we have reached the end of the corner
                    in_element__ = false;                                           % We are, so move to the next track element
                end
            end
            
            if self.velocity > self.max_velocity                                    % Check if we have exceeded our last max velocity
                self.max_velocity = self.velocity;                                  % Update max velocity reached
            end
        end
    end
end