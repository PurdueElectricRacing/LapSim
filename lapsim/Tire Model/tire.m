classdef tire
    properties
        Fz;         %current load (N)
        Fz_nom;     %nominal load (N)
        
        PC_x;       %longitudinal Pacejka coefficients
        PC_y;       %lateral Pacejka coefficients
    end
    methods
        function obj = tire(Fz_nom)
            % constructor
            obj.Fz_nom = Fz_nom;
            
            %get the Pacejka coefficients
            PC_x = magic_formula_straight(Fz_tire, CA, lambda_mu);
            PC_y = magic_formula_corner(Fz_tire, CA, lambda_mu);
        end
    end
end