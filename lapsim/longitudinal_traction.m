function [ max_torque_traction ] = longitudinal_traction( mass, acceleration, Cg_z, wheel_base, fdr, tire_radius, Cl, air_density, velocity, Al, coeff_f, driveline_eff )
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Summary: Calculates the maximum torque available at the wheels

%Inputs: 

%static car characteristics from input file
%instananeous car acceleration
%instantaneous car velocity


%Outputs: maximum instantaneous torque available at the wheels

%Variables:
%shaft_torque: torque at the motor shaft
%max_torque_traction: torque at the wheels
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
delta_weight = mass * acceleration * Cg_z / wheel_base; %simple longitudinal weight transfer

max_tractive_force = (coeff_f * (((mass / 2) * 9.8 + delta_weight) - (Cl * air_density * velocity^2 * Al * 0.5)*0.5)); 
%^^ max force provided by rear wheels at terminal velocity assuming equal
%   longitudinal weight distribution across the car and power limited
%   acceleration
tire_radius = tire_radius * 0.0254; % converts to m
shaft_torque = max_tractive_force * tire_radius; %[Nm]
max_torque_traction = (shaft_torque * (1/driveline_eff)) / fdr; %[Nm]

end

