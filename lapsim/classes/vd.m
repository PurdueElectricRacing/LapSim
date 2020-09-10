classdef vd < handle
    properties
        %Inputs
        susp_params
        veh_params
        acceleration
        vehicle_mass
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
            obj.vehicle_mass = 200;
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
        
        function [fr, fl, rr, rl] = weight_transfer_variable(self,rho)
            %calculates the change in force on each tire due to
            %accelerations
            Cgz = self.veh_params.Cg_xyz(3);
            wheelbase = self.veh_params.wheelbase;
            track_front = self.veh_params.track_front;
            track_rear = self.veh_params.track_rear;
            mass = self.veh_params.mass;
            
            syms v_max;
            lat_accel_capacity = v_max^2 / rho;
            
            longitudinal =  0; %self.Accel(1)*mass*Cgz/wheelbase; positive rear
            front_lateral = lat_accel_capacity*mass*Cgz/track_front; %positive right
            rear_lateral = lat_accel_capacity*mass*Cgz/track_rear;
            
            fr = -longitudinal + front_lateral + (.25 * mass);
            fl = -longitudinal - front_lateral + (.25 * mass);
            rr = longitudinal + rear_lateral + (.25 * mass);
            rl = longitudinal - rear_lateral + (.25 * mass);
        end
        
        function [max_speed_table] = max_corner_speed(track_table)
            %determines maximum corner speed for each corner and outputs to
            %table based on track input table
            
            tire_coef = 1.5;
            i=1;
            for track_inputs i=1:length(track_table)
                if track_table(i,3) ~= 0                                      %true if line in track table is a corner not a straight
                    syms v_max;                                             %variable to solve for
                    rho = track_table(i,3);                                    %corner radius for current corner
                    lat_accel = v_max^2 / rho;                              %expression for lateral acceleration in terms of v_max
                    [fr, fl, rr, rl] = weight_transfer_variable(self, rho);   %determine weight transfer in terms of v_max
                    acceleration_eq = (tire_coef * (fr + fl + rr + rl)) / self.mass == lat_accel;     %available traction @ v_max == lateral accel @v_max: this is just an equation that is solved for the max velocity in the next line
                    max_speed_table(i) = vpasolve(accceleration_eq, v_max);        %solve for v_max in this corner and put in max_speed_table
                    i=i+1;                                                %increment index value
                else                                                     %if line of track table is a straight
                    max_speed_table(i) = 0;                              %placeholder for v_max on straightaway
                    i=i+1;                                                %increment index value
                end
            end
        end
  
        function = vd_main(aero, drivetrain);
            %inputs: "max_motor_torque" instantanously at wheel center for current conditions in Newton-meters, system time step (seconds), car mass (kg), track table
            %outputs: current velocity, motor_torque_used for one wheel given time step
            wheel_radius = 0.2286; %wheel diameter in meters
            coef = 1.5; %tire coefficient of friction
            num_motor = 4; %%%%%%%%%%%%number of hub motors 2- rear wheel drive, 4 - all wheel drive
            max_motor_torque = drivetain.max_motor_torque; %%%%%%get input of max instanteous torque

            radius = track_table(k,3); %get radius of turn
            if radius ~= 0
                dist_corner = 0;
                curr_velocity = max_speed_table(k);
                corner = 0; %not in a corner
            else
                corner = 1; % in a corner
            end

            straight_length = track_table(k,2); %pass length of straight
            dist_corner = track_table(k,3) - (time_step * curr_velocity); %calc distance to corner with straight length and distance change
            if dist_corner <= 0
                k = k+1; %increment to next part of track
            end

            low_velocity = sqrt(curr_velocity^2 + (-19.62)*dist_corner); %determine lowest acheivable velocity at start of turn with 2G of braking

            RR_force = (.005 + (1/1.379) * (0.01+0.0095*((curr_velocity*1000)/100)^2))*(vehicle_mass*9.81); %calc current rolling resistance
            drag_force = aero.drag_force; %call aero "aero_calc" function and get output of self.drag_force

            if curr_velocity <= max_speed_table(k) && max_speed_table(k) >= low_velocity && corner == false %checks if car can still accelerate and be able to brake in time
                motor_force = max_motor_torque / wheel_radius; %calc acceleration force of car
                fric_force = coef * vehicle_mass/4; %
                if motor_force < fric_force
                    accel_force = motor_force * num_motor;
                    motor_torque_used = max_motor_torque;
                else
                    accel_force = fric_force * num_motor;
                    motor_torque_used = fric_force / wheel_radius;
                end

                curr_velocity = curr_velocity +(((accel_force*num_motor) - RR_force - drag_force)/vehicle_mass * time_step); %accelerate car

            elseif curr_velocity > max_speed_table(k) && corner == false %checks if car needs to brake
                curr_velocity = curr_velocity + (curr_velocity - (19.62*time_step)); %brakes car at -2G

            else %checks if car should stay going same velocity b/c its in a turn
                motor_torque_used = (RR_force + drag_force) / num_motor * wheel_radius; %overcome rolling resistance and drag
            end
        end
    end
end