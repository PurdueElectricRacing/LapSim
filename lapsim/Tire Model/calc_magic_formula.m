function [val] = calc_magic_formula(coeff,S)
%%
% Summary: Calculates the Pacejka magic formula model for a given slip angle or
%  ratio.
% Inputs:
%  coeff - the coefficient vector [B, C, D, E]
%  S - slip ratio or angle (angle in radians)
% Outputs:
%  val: Fy or Fx
% Note: a vertical shift might be required but is not included.
% See Race Car Design pg 131
%%
val = coeff(3)*sin(coeff(2)*atan(coeff(1)*S-coeff(4)*(coeff(1)*S-atan(coeff(1)*S))));
end