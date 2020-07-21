classdef tires < handle  % must include "handle" in order to pass and return 
                        % the class to functions.  Read about value vs
                        % handle classes: https://www.mathworks.com/help/matlab/matlab_oop/comparing-handle-and-value-classes.html
    properties
        Fz;         %current load (N)
        %Fz_nom;     %nominal load (N)
        
        Fx_mf;      %current tractive forces
        Fy_mf;      % calculated from the magic formula
        Mz;         %aligning torque
        
        Fx_mnc;     %combined fx/fy calculated from MNC model
        Fy_mnc; 
        
        sr;         %slip ratio: + accelerating
        sa;         %slip angle: + vehicle steering to the right/tire 
                    % sliding to the left
        
        steer_angle = 0; %steering angle, relative to chassis (radians)
                         % Right turn is positive.
        
        u_vehicle;   %[vehicle x (forward) velocity; y (right) velocity]
        v_tire;      %[tire longitudinal (forward) velocity; lateral (right) velocity]
        angular_vel; %rotational velocity of the tire (rad/s - forward positive)
        
        PC_x;       % current Pacejka coefficients given load Fz
        PC_y;       % lateral
        PC_z;       % aligning torque
        
        PC_x_mat;   %longitudinal Pacejka coefficients (50; 150; 200; 250) lbs
        PC_y_mat;   %lateral Pacejka coefficients (50; 100; 150; 200; 250) lbs
        PC_z_mat;   %aligning torque PCs
        
        tire_radius = 18/2 * 0.0254; % 18 inch diameter
    end
    
    methods
        function obj = tires()
            % constructor. this function is called when an instance is
            % created
            
            %get the Pacejka coefficients for all loads in the test data
            [obj.PC_x_mat, obj.PC_y_mat, obj.PC_z_mat] = get_variable_load_params();
            
            %obj.Fz_nom = Fz_nom;
        end
        
        function [PC_x_interp, PC_y_interp, PC_z_interp] = interp_PCs(self)  
            %interpolates PCs between two nominal loads. Fz must be [N]
            Fz_lbs = self.Fz / 4.448; %get the load in lbs
            switch true
                case Fz_lbs <= 100
                    PC_x_low  = self.PC_x_mat(1,:);
                    PC_x_high = self.PC_x_mat(2,:);
                    Fz_x_low  = 50;
                    Fz_x_high = 150;
                    
                    PC_y_low  = self.PC_y_mat(1,:);
                    PC_y_high = self.PC_y_mat(2,:);
                    PC_z_low  = self.PC_z_mat(1,:);
                    PC_z_high = self.PC_z_mat(2,:);
                    Fz_y_low  = 50;
                    Fz_y_high = 100;
                    
                case Fz_lbs > 100 & Fz_lbs <= 150
                    PC_x_low  = self.PC_x_mat(1,:);
                    PC_x_high = self.PC_x_mat(2,:);
                    Fz_x_low  = 50;
                    Fz_x_high = 150;
                    
                    PC_y_low  = self.PC_y_mat(2,:);
                    PC_y_high = self.PC_y_mat(3,:);
                    PC_z_low  = self.PC_z_mat(2,:);
                    PC_z_high = self.PC_z_mat(3,:);
                    Fz_y_low  = 100;
                    Fz_y_high = 150;
                    
                case Fz_lbs > 150 & Fz_lbs <= 200
                    PC_x_low  = self.PC_x_mat(2,:);
                    PC_x_high = self.PC_x_mat(3,:);
                    Fz_x_low  = 150;
                    Fz_x_high = 200;
                    
                    PC_y_low  = self.PC_y_mat(3,:);
                    PC_y_high = self.PC_y_mat(4,:);
                    PC_z_low  = self.PC_z_mat(3,:);
                    PC_z_high = self.PC_z_mat(4,:);
                    Fz_y_low  = 150;
                    Fz_y_high = 200;
                 
                case Fz_lbs > 200
                    PC_x_low  = self.PC_x_mat(3,:);
                    PC_x_high = self.PC_x_mat(4,:);
                    Fz_x_low  = 200;
                    Fz_x_high = 250;
                    
                    PC_y_low  = self.PC_y_mat(4,:);
                    PC_y_high = self.PC_y_mat(5,:);                    
                    PC_z_low  = self.PC_z_mat(4,:);
                    PC_z_high = self.PC_z_mat(5,:);
                    Fz_y_low  = 200;
                    Fz_y_high = 250;
            end
            kx = (Fz_lbs - Fz_x_low) / (Fz_x_high - Fz_x_low);
            ky = (Fz_lbs - Fz_y_low) / (Fz_y_high - Fz_y_low);
            kz = (Fz_lbs - Fz_y_low) / (Fz_y_high - Fz_y_low);
            
            PC_x_interp = PC_x_high * kx + PC_x_low * (1-kx);
            PC_y_interp = PC_y_high * ky + PC_y_low * (1-ky);
            PC_z_interp = PC_z_high * kz + PC_z_low * (1-kz);
            
            self.PC_x = PC_x_interp;
            self.PC_y = PC_y_interp;
            self.PC_z = PC_z_interp;
        end
        
        function v_tire = get_tire_velocity_vector(self)
            b = self.steer_angle;
            rotation_matrix = [cos(b) sin(b); -sin(b) cos(b)]; %tire = R * vehicle
            v_tire = rotation_matrix * self.u_vehicle;
            self.v_tire = v_tire;
        end
        
        function sr = get_sr(self)
            %SAE J670 slip ratio definition: S = (omega*Re / V*cosa) - 1
            omega = self.angular_vel; %angular velocity of the tire
            Re = self.tire_radius;    %effective tire radius
            Vcosa = self.v_tire(1);   %Vcosa = vx
            sr = omega*Re / Vcosa - 1;
            self.sr = sr;
        end
        
        function sa = get_sa(self)
            vx = self.v_tire(1); %tire velocity, forward positive
            vy = self.v_tire(2); % RIGHT positive
            
            sa = atan(-vy/vx); %left tire motion, positive sa
            self.sa = sa;
        end
        
        function [fx, fy, mz] = calc_magic_formula(self)
            %Calculates the Pacejka magic formula model for a given slip
            %parameter.
            fx = self.PC_x(3)*sin(self.PC_x(2)*atan(self.PC_x(1)*self.sr-self.PC_x(4)*(self.PC_x(1)*self.sr-atan(self.PC_x(1)*self.sr))));
            fy = self.PC_y(3)*sin(self.PC_y(2)*atan(self.PC_y(1)*self.sa-self.PC_y(4)*(self.PC_y(1)*self.sa-atan(self.PC_y(1)*self.sa))));
            mz = self.PC_z(3)*sin(self.PC_z(2)*atan(self.PC_z(1)*self.sa-self.PC_z(4)*(self.PC_z(1)*self.sa-atan(self.PC_z(1)*self.sa))));
            self.Fx_mf = fx;
            self.Fy_mf = fy;
            self.Mz = mz;
        end
            
        function [fx, fy] = calc_MNC(self)
            %calculates combines cornering and traction using the Modified
            %Nicolas-Comstock model
            
            %find the slope of the magic formula at 0 (=B*C*D)
            Cs = self.PC_x(1) * self.PC_x(2) * self.PC_x(3);
            Ca = self.PC_y(1) * self.PC_y(2) * self.PC_y(3);
            
            fy_a = abs(self.Fy_mf);  %equations only work for positive forces
            fx_s = abs(self.Fx_mf);
            
            SR = abs(self.sr);
            SA = abs(self.sa);
            
            %calculate model
            fx_as = fx_s.*fy_a ./ sqrt( (SR.*fy_a).^2 + (tan(SA).*fx_s).^2 ) .* sqrt( (SR.*Ca).^2 + ((1-SR).*cos(SA).*fx_s).^2 ) ./ Ca;
            fy_as = fx_s.*fy_a ./ sqrt( (SR.*fy_a).^2 + (tan(SA).*fx_s).^2 ) .* sqrt( ((1-SR).*cos(SA).*fy_a).^2 + (sin(SA).*Cs).^2 ) ./ (cos(SA).*Cs);
            self.Fx_mnc = fx_as * sign(self.Fx_mf);  %make sure the signs are correct
            self.Fy_mnc = fy_as * sign(self.Fy_mf);
        end
        
        function update_tire(self, Fz, steering_angle, veh_vel_vector, angular_vel)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % runs all tire functions in the correct order.
            % Inputs:
            %  Fz: tire normal force [N]
            %  steering_angle: tire steering angle [radians, + is right]
            %  veh_vel_vector: vehicle velocity: [Vx; Vy]
            %  angular_vel: angular velocity of the tire [rad/s]
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            self.Fz = Fz;
            self.interp_PCs;
            self.steer_angle = steering_angle;
            self.u_vehicle = veh_vel_vector;
            self.get_tire_velocity_vector;
            self.angular_vel = angular_vel;
            self.get_sr;
            self.get_sa;
            self.calc_magic_formula;
            self.calc_MNC;
        end
        
        function plot_magic_formula(self, type)
            % test function. plots the magic formula.
            % type - set to 'x', 'y', or 'xy' to choose what to plot
            if ismember('x', type)
                s = -0.3:0.01:0.3;
                fx = calc_magic_formula(self.PC_x, s);
                figure(3);
                hold on;
                plot(s,fx,"b");
            end
            if ismember('y', type)
                a = -0.3:0.01:0.3;
                fy = calc_magic_formula(self.PC_y, a);
                figure(2);
                hold on;
                plot(a,fy,"r");
            end
            if ismember('z', type)
                a = -0.3:0.01:0.3;
                mz = calc_magic_formula(self.PC_z, a);
                figure(4);
                hold on;
                plot(a,mz,"g");
            end
        end
    end
end