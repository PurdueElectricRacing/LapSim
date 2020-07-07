function [Fx, Fy] = MNC(s,a,PC_x,PC_y)
%%%%%%%%%%%%%%%%%%%
% Summary: Calculates the Modified Nicolas-Comstock model for combined
% steering and traction (aka the tire force ellipse)
% Inputs:
%  s - slip ratio
%  a - slip angle (degrees)
%  PC_x - vector of longitudingal Pacejka coefficients
%  PC_y - vector of lateral Pajecka coefficients
% Outputs:
%  [Fx, Fy] - tire forces
% Note:
% x refers to longitudinal or forward direction
% y refers to lateral or left direction
% see the paper "The Tire-Force Ellipse (Friction Ellipse) and Tire
% Characteristics" (Brach and Brach, 2011)
%%%%%%%%%%%%%%%5

a = a * pi/180; %convert degrees to radians

%find Fx and Fy from the Pacejka model
Fx_mf = calc_magic_formula(PC_x, s);
Fy_mf = calc_magic_formula(PC_y, a);

%find the slope of the magic formula at 0 (=B*C*D)
Cs = PC_x(1) * PC_x(2) * PC_x(3);
Ca = PC_y(1) * PC_y(2) * PC_y(3);

%MNC model
Fx = Fx_mf.*Fy_mf ./ sqrt( (s.*Fy_mf).^2 + (tan(a).*Fx_mf).^2 ) .* sqrt( (s.*Ca).^2 + ((1-abs(s)).*cos(a).*Fx_mf).^2 ) ./ Ca;
Fy = Fx_mf.*Fy_mf ./ sqrt( (s.*Fy_mf).^2 + (tan(a).*Fx_mf).^2 ) .* sqrt( ((1-abs(s)).*cos(a).*Fy_mf).^2 + (sin(a).*Cs).^2 ) ./ (cos(a).*Cs);