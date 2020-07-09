classdef sensors
    properties
        % Inputs
        vd
        tires
        tctv
        motor
        lv
        hv
        driver
        driveline
        cooling
        battery
        aero
        
        % Outputs
        data
        config
    end
    
    methods
        function obj = sensors(v, t, tc, mot, l, h, dr, drlne, cool, batt, ae, cfg)
            % Sensor class constructor
            % Input vehicle classes and sensor configuration parameters
            % Only call once all other classes have been constructed
            if nargin == 12
                obj.vd = v;
                obj.tires = t;
                obj.tctv = tc;
                obj.motor = mot;
                obj.lv = l;
                obj.hv = h;
                obj.driver = dr;
                obj.driveline = drlne;
                obj.cooling = cool;
                obj.battery = batt;
                obj.aero = ae;
                obj.config = cfg;
            end
        end
    end
end

