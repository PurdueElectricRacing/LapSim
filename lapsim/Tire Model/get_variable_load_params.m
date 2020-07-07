function [Ax, Ay] = get_variable_load_params()
%%%
% Summary: Calculates magic formula parameters for every load case in the
% data, for interpolation
% Outputs:
%  Ax - matrix: each row is [Bx, Cx, Dx, Ex] for loads [50; 150; 200; 250]
%  Ay - matrix: each row is [By, Cy, Dy, Ey] for [50; 100; 150; 200; 250]
%%%

Ax(1,:) = magic_formula_straight(4.448*50,  0, 1, 50 );
Ax(2,:) = magic_formula_straight(4.448*150, 0, 1, 150);
Ax(3,:) = magic_formula_straight(4.448*200, 0, 1, 200);
Ax(4,:) = magic_formula_straight(4.448*250, 0, 1, 250);

Ay(1,:) = magic_formula_corner(4.448*50,  0, 1, 50 );
Ay(2,:) = magic_formula_corner(4.448*100, 0, 1, 100);
Ay(3,:) = magic_formula_corner(4.448*150, 0, 1, 150);
Ay(4,:) = magic_formula_corner(4.448*200, 0, 1, 200);
Ay(5,:) = magic_formula_corner(4.448*250, 0, 1, 250);
end