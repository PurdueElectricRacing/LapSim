function [val] = calc_magic_formula(coeff,S)
val = coeff(3)*sin(coeff(2)*atan(coeff(1)*S-coeff(4)*(coeff(1)*S-atan(coeff(1)*S))));
end