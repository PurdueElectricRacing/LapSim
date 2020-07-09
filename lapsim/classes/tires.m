classdef tires < handle  % must include "handle" in order to pass and return 
                        % the class to functions.  Read about value vs
                        % handle classes: https://www.mathworks.com/help/matlab/matlab_oop/comparing-handle-and-value-classes.html
    properties
        Fz;         %current load (N)
        Fz_nom;     %nominal load (N)
        
        Fx;         %current tractive forces
        Fy;         %
        
        PC_x_mat;   %longitudinal Pacejka coefficients (50; 150; 200; 250) lbs
        PC_y_mat;   %lateral Pacejka coefficients (50; 100; 150; 200; 250) lbs
        PC_z_mat;   %aligning torque PCs
        
        PC_x;       % current Pacejka coefficients given load Fz
        PC_y;       % lateral
        PC_z;       % aligning torque
    end
    
    methods
        function obj = tires()
            % constructor. this function is called when an instance is
            % created
            
            %get the Pacejka coefficients for all loads in the test data
            [obj.PC_x_mat, obj.PC_y_mat, obj.PC_z_mat] = get_variable_load_params();
            
            %obj.Fz_nom = Fz_nom;
        end
        
        function [PC_x_interp, PC_y_interp, PC_z_interp] = interp_PCs(self, Fz)  
            %interpolates PCs between two nominal loads. Fz must be [N]
            Fz_lbs = Fz / 4.448; %get the load in lbs
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
            PC_z_interp = PC_z_high * kz + PC_y_low * (1-kz);
            
            self.PC_x = PC_x_interp;
            self.PC_y = PC_y_interp;
            self.PC_z = PC_z_interp;
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
        
        %function MNC(
    end
end