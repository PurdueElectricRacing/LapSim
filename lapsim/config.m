%All variables require a ';' at the end of the line to prevent being
%printed
%% Simulation
time_step = 0.1; %time step [s]
power_limit = 80000; %competition power limit in KW
%% Track
%remove the '%' infront of the track you wish to simulate.

acceleration_run;

%autox;
%skidpad;
%endurance;

%% Overall Vehicle
vehicle_name = 'PER 2016'; %vehicle name, must be in single quotations
mass = 272; %mass including driver mass! [kg]
Cd = 0.6; %Coefficient of drag
Cl = 2; %lift coefficient
Af = 1.3; %Frontal Area [m^2]
Al = 2.5; %Lateral Area [m^2]
air_density = 1.225; %Density of the air, 1.225 is at sea level and 15C [kg/m^3]
Cg_z = 0.45; %center of gravity Z
wheel_base = 1.5748;
%% Suspension & Tires
Cr = 0.03; %Rolling resistance coefficient
tire_radius = 9; %tire radius in inches
lateral_g = 1.4; %maximum lateral cornering force [g]
max_braking_force = 1.5; %maximum braking force [g]
%longitudinal_friction = 0.9; %longitudinal coefficient of friction
coeff_f = 2; %longitudinal friction
%% Driveline
fdr = 3.6; %Final drive ratio - for 4:1 enter 4
driveline_eff = 0.85; %Driveline efficiency

%% Motor
max_motor_power = 80000; %aidan add [W]
max_motor_current = 320; %aidan add [A] from data sheet

motor_eff = 0.97; %Motor efficiency
kt = 0.50; %motor torque constant [Nm/A]
KV = 22; %motor rotations per volt constant [RPM/V]
motor_R = 0.0057; %motor resistance [m Ohms]
motor_table = [0,200,400,600,800,1000,1200,1400,1600,1800,2000,2200,2400,2600,2800,3000,3200,3400,3600,3800,4000,4200,4400,4600,4800,5000,5200,5400,5600,5800,6000;
                140,140,140,140,140,140,140,140,140,139,139,139,138,138,138,136,136,135,135,135,135,135,134,131,130,129,125,124,120,118,115]; %table of motor rpm vs torque

%% Battery
Ah = 22; %Total battery amp hours [Ah]
batt_V_max = 300; %Max battery voltage [V]
R_int = 0.002; %Cell internal resistance [Ohms]
cell_s = 72; %number of cells in series
cell_p = 4; %number of cells in parrallel
pack_R = (R_int * cell_s)/cell_p; %pack internal resistance
battery_table = [4.1667 100; %cell ocv at cell SOC
                 3.0 0];
batt_current_limit = 400; %aidan add
%%
fprintf('Vehicle parameters loaded\n')
