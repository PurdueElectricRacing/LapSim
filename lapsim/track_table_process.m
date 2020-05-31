function [ track_table ] = track_table_process( track_table, lateral_g, mass, air_density, Af, Cd, Cr, Cl, Al )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Summary: Calculates maximum cornering velocity for each track turn based
%on the inputs

%Inputs:

%track_table: track for desired dynamic event
%lateral_g: maximum lateral cornering force [g]
%mass: total car mass with driver [kg]
%air_density: density of air [kg/m^3]
%Af: car frontal area [m^2]
%Al: car lateral area [m^2]
%Cd: coeffiecient of drag 
%Cl: coefficient of lift
%Cr: coefficient of rolling resistance

%Outputs: Four column vector of the form 
%         [ corner (bool), section distance (m), corner radius (m), max corner speed (m/s) ] 

%Variables:

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

R = (mass * 9.8); %total vehicle weight



[rows, ~] = size(track_table);
for i = 1:rows
    if track_table(i, 1) == 1 %if current section is a turn
        r = track_table(i, 3) / 2;
        track_table(i, 4) = ((2*r*(R*(Af^2*Cd^2*lateral_g^2*r^2*air_density^2 - 4*Cr^2*mass^2 + 4*lateral_g^2*mass^2)^(1/2) + Al*Cl*Cr^2*R*r*air_density - Al*Cl*lateral_g^2*R*r*air_density - Af*Cd*Cr*R*r*air_density))/(Af^2*Cd^2*r^2*air_density^2 - 2*Af*Al*Cd*Cl*Cr*r^2*air_density^2 + Al^2*Cl^2*Cr^2*r^2*air_density^2 - Al^2*Cl^2*lateral_g^2*r^2*air_density^2 + 4*mass^2))^(1/2);
        %^^ track_table(i, 4) is maximum cornering velocity
    else
        track_table(i, 4) = 0;
    end
end

end