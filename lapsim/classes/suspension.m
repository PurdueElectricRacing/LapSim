classdef suspension < handle
    %class for suspension of individual wheel assembly
    properties
        wheel_fz;   %current load on the wheel
        wheel_disp; %displacement of the wheel upright;
        wheel_vel;  %derivative of upright displacement
        wheel_acc;  %2nd deriv of upright displacement
        
        shock_f;    %force exerted by the spring/damper
        shock_disp; %displacement of the shock;
        shock_vel;  %derivative of shock displacement
        shock_acc;  %2nd deriv of shock displacement
        
        Ks; %spring rate
        Bs; %damping rate
        motion_ratio; %average motion ratio of suspension - wheel disp / shock disp
        m_unsp; %unsprung mass of this wheel assembly
        m_sp; %sprung mass load on this wheel (1/4 of total sprung mass?)
        
        tire; %tire class
    end
    
    methods
        function obj = suspension(raw_vals)
            obj.Ks              = raw_vals(1);
            obj.Bs              = raw_vals(2);
            obj.motion_ratio    = raw_vals(3);
            obj.m_unsp          = raw_vals(4);
            obj.m_sp            = raw_vals(5);
            
            obj.tire = tires(); %create the tire class for this wheel
        end
        
        function shock_force(self)
            self.shock_disp = self.wheel_disp / self.motion_ratio;
            self.shock_vel = self.wheel_vel / self.motion_ratio;
            self.shock_acc = self.wheel_acc / self.motion_ratio;
            self.shock_f = self.shock_disp * self.Ks + self.shock_vel * self.Bs;
            
            self.wheel_fz = self.shock_f / self.motion_ratio + self.wheel_acc * self.m_unsp;
        end
    end
end