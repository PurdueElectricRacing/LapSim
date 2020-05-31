function [ traction ] = traction_simple( normal_load, cf )
%This function finds the traction of a single wheel based on the formula
%Traction = normal load * coefficient of friction
%traction: The available traction at the wheels
%normal_load: the normal load on the wheel
%cf: coefficienct of friction

traction = normal_load*cf;


end

