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
                    syms v_max;                                                     % Variable to solve for
                    rho = track_table(i,3);                                         % Corner radius for current corner
                    lat_accel = v_max^2 / rho;                                      % Expression for lateral acceleration in terms of v_max
                    [fr, fl, rr, rl] = weight_transfer_variable(self, rho);         % Determine weight transfer in terms of v_max
                    acceleration_eq = (self.tire_coef * (fr + fl + rr + rl)) / self.vehicle_mass == lat_accel;   % Available traction @ v_max == lateral accel @v_max: this is just an equation that is solved for the max velocity in the next line
                    max_speed_table(i) = vpasolve(v_max, acceleration_eq);    % Solve for v_max in this corner and put in max_speed_table
                else                                                                % If line of track table is a straight
                    max_speed_table(i) = 0;                                         % Placeholder for v_max on straightaway
                end
            end
        end
  
        function [motor_torque_used, curr_velocity] = vd_main(self, aero, drivetrain, track_table, max_table, dt)
            %inputs: "max_motor_torque" instantanously at wheel center for current conditions in Newton-meters, system time step (seconds), car mass (kg), track table
            %outputs: current velocity, motor_torque_used for one wheel given time step
            max_motor_torque = drivetrain.max_motor_torque; %%%%%%get input of max instanteous torque

            curr_velocity = self.velocity;

            radius = cell2mat(track_table(self.k,3)); %get radius of turn
            if radius ~= 0
                corner = 0; %not in a corner
            else
                corner = 1; % in a corner
                curr_velocity = max_table(self.k);
            end

            dist_corner = cell2mat(track_table(self.k, 2)) - (dt * curr_velocity); %calc distance to corner with straight length and distance change
            if dist_corner <= 0
                self.k = self.k + 1; %increment to next part of track
            end

            low_velocity = sqrt(curr_velocity^2 + (-19.62)*dist_corner); %determine lowest acheivable velocity at start of turn with 2G of braking

            RR_force = (.005 + (1/1.379) * (0.01+0.0095*((curr_velocity*1000)/100)^2))*(self.vehicle_mass*9.81); %calc current rolling resistance
            drag_force = aero.drag_force; %call aero "aero_calc" function and get output of self.drag_force

            if curr_velocity <= max_table(self.k) && max_table(self.k) >= low_velocity(1) && corner == false %checks if car can still accelerate and be able to brake in time
                motor_force = max_motor_torque / self.wheel_radius; %calc acceleration force of car
                fric_force = self.tire_coef * self.vehicle_mass / 4; %
                if motor_force < fric_force
                    accel_force = motor_force * self.num_motor;
                    motor_torque_used = max_motor_torque;
                else
                    accel_force = fric_force * self.num_motor;
                    motor_torque_used = fric_force / self.wheel_radius;
                end

                curr_velocity = curr_velocity +(((accel_force*self.num_motor) - RR_force - drag_force)/self.vehicle_mass * dt); %accelerate car

            elseif curr_velocity > max_table(self.k) && corner == false %checks if car needs to brake
                curr_velocity = curr_velocity + (curr_velocity - (19.62*dt)); %brakes car at -2G
                motor_torque_used = 0;

            else %checks if car should stay going same velocity b/c its in a turn
                motor_torque_used = (RR_force + drag_force) / self.num_motor * self.wheel_radius; %overcome rolling resistance and drag
            end

            self.velocity = curr_velocity;
        end
    end
end