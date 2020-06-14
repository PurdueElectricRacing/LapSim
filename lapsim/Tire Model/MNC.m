function [Fx, Fy] = MNC(s,a,Fz_tire,CA,lambda_mu)
%%
% Summary: Calculates the Modified Nicolas-Comstock model for combined
% steering and traction (aka the tire force ellipse)
% Inputs:
%  s - slip ratio
%  a - slip angle (degrees)
%  Fz_tire - the normal force on the tire (N)
%  CA - camber angle (degrees)
%  lambda_mu - friction scaling factor (default = 1)
% Outputs:
%  [Fx, Fy] - tire forces
% Note:
% x refers to longitudinal or forward direction
% y refers to lateral or left direction
% see the paper "The Tire-Force Ellipse (Friction Ellipse) and Tire
% Characteristics" (Brach and Brach, 2011)
%%

a = a * pi/180; %convert degrees to radians

%get the Pacejka coefficients
PC_x = magic_formula_straight(Fz_tire, CA, lambda_mu);
PC_y = magic_formula_corner(Fz_tire, CA, lambda_mu);

%find Fx and Fy from the Pacejka model
Fx_mf = calc_magic_formula(PC_x, s);
Fy_mf = calc_magic_formula(PC_y, a);

%find the slope of the magic formula at 0 (=B*C*D)
Cs = PC_x(1) * PC_x(2) * PC_x(3);
Ca = PC_y(1) * PC_y(2) * PC_y(3);

%MNC model
Fx = Fx_mf.*Fy_mf.*s ./ sqrt( (s.*Fy_mf).^2 + (tan(a).*Fx_mf).^2 ) .* sqrt( (s.*Ca).^2 + ((1-abs(s)).*cos(a).*Fx_mf).^2 ) ./ (s.*Ca);
Fy = Fx_mf.*Fy_mf.*tan(a) ./ sqrt( (s.*Fy_mf).^2 + (tan(a).*Fx_mf).^2 ) .* sqrt( ((1-abs(s)).*cos(a).*Fy_mf).^2 + (sin(a).*Cs).^2 ) ./ (sin(a).*Cs);