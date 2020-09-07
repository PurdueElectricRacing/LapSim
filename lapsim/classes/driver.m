classdef driver
    properties
        % Inputs
        % None, treat this as an interface
        
        % Outputs
        steering_angle
        throttle
        brake
        brake_bal
        regen_en
        tc_en
        lc_en
    end
    
    methods
        function obj = driver(raw_vals)
            obj.regen_en  = raw_vals(1);
            obj.tc_en     = raw_vals(2);
            obj.lc_en     = raw_vals(3);
            obj.brake_bal = raw_vals(4);
        end
        
        % The lapsim functionality hasn't extended to need the driver yet.
        % Hence, there's nothing here quite yet.
    end
end

