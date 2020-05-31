
clear;
clc;


%%Things I need to Add:
%coefficient of downforce
%LV load
%equivalent mass
%regen
%heat/cooling for battery cells
%proper battery voltage model
%post processing gui


config; %loads vehicle characteristic file variables
fprintf('Simulation of %s initiated \n', vehicle_name)

%intialize variables

i = 1; %counter loop [unitless] 
motor_RPM(i) = 0; % [RPM]
batt_ocv(i) = batt_V_max; %battry open circuit voltage [volts]
battery_SOC(i) = 100; %assume start fully charged [%]
batt_current(i) = 0; %instantaneous battery current [amps]
acceleration(i) = 0; %instantaneous car acceleration [m/s^2]
velocity(i) = 0; %instantaneous car velocity [m/s]
distance_run(i) = 0; %meters
braking_distance = 0; %meters
start_i = 0; %unitless
lap_count = 0; %laps completed
air_density = 1.225; %[kg/m^3]


%aidan add
state = 'z'; %initialize car state variable
last_state = 'z';
section_distances(i) = 0;
braking_distances(i) = 0;
max_torque_traction(i) = 0;

%pre-process motor and battery data
motor_table_interp = process_motor_table(motor_table); %RPM/torque curve in the form [ RPM, Torque ]
battery_table_interp = process_battery_table(battery_table); %SOC/voltage curve in the form [ SOC, Voltage ]
track_table = track_table_process(track_table, lateral_g, mass, air_density, Af, Cd, Cr, Cl, Al);

%run loop
[rows, columns] = size(track_table); %row and column count of track table
for lap_count = [1:laps]; %loops through number of laps // laps given in event track
    for corner_count = [1:rows]; %number of corners in a given lap
        
        %debug print statement:
        %fprintf('\ncorner count: %d Lap count: %d', corner_count, lap_count);
        
        braking_distance = 0; %distance needed to brake to desired speed
        section_distance = 0; %distance traveled in each track section
        brake_started = 0; %bool value
        start_i = i;
        [vehicle_state, velocity(i)] = state_selection(track_table, corner_count, section_distance, braking_distance, velocity, i, brake_started);
        while strcmp('finish', vehicle_state) == 0; %continues loop until corner is completed
            %case 'finish' signifies end of track section
            switch vehicle_state
                case 'last_straight'
                %vehicle is in final section which is a straight, it
                %accelerates throughout
                    state = 'last straight';
                    pedal = 1;
                    max_torque_traction(i) = longitudinal_traction(mass, acceleration(i), Cg_z, wheel_base, fdr, tire_radius, Cl, air_density, velocity(i), Al, coeff_f, driveline_eff);
                    [batt_current(i+1), motor_torque(i+1)] = motor( batt_ocv(i), batt_current(i), pack_R, motor_table_interp, motor_RPM(i), batt_current_limit, power_limit, max_torque_traction(i), max_motor_power, motor_eff, max_motor_current);
                    [battery_SOC(i+1), batt_ocv(i+1)] = battery(batt_current(i), battery_SOC(i), Ah, battery_table_interp, time_step, cell_s, batt_V_max);
                    wheel_torque(i+1) = driveline(motor_torque(i+1), fdr, driveline_eff);
                    [acceleration(i+1)] = acceleration_calc(wheel_torque(i+1), Cd, Af, Cr, air_density, mass, tire_radius, velocity(i));
                    [velocity(i+1), distance_run(i+1), motor_RPM(i+1)] = velocity_distance(acceleration(i), velocity(i), distance_run(i), time_step, tire_radius, fdr, track_table, corner_count);
                    % vehicle is accelerating along a straight with a corner ahead
                case 'straight_accel'
                    state = 'straight accel';
                    pedal = 1;
                    max_torque_traction(i) = longitudinal_traction(mass, acceleration(i), Cg_z, wheel_base, fdr, tire_radius, Cl, air_density, velocity(i), Al, coeff_f, driveline_eff);
                    [batt_current(i+1), motor_torque(i+1)] = motor( batt_ocv(i), batt_current(i), pack_R, motor_table_interp, motor_RPM(i), batt_current_limit, power_limit, max_torque_traction(i), max_motor_power, motor_eff, max_motor_current);
                    [battery_SOC(i+1), batt_ocv(i+1)] = battery(batt_current(i), battery_SOC(i), Ah, battery_table_interp, time_step, cell_s, batt_V_max);
                    wheel_torque(i+1) = driveline(motor_torque(i+1), fdr, driveline_eff);
                    [acceleration(i+1)] = acceleration_calc(wheel_torque(i+1), Cd, Af, Cr, air_density, mass, tire_radius, velocity(i));
                    [velocity(i+1), distance_run(i+1), motor_RPM(i+1)] = velocity_distance(acceleration(i), velocity(i), distance_run(i), time_step, tire_radius, fdr, track_table, corner_count);
                    braking_distance = braking_distance_calc(velocity(i + 1), track_table(corner_count + 1, 4), max_braking_force);
                    % vehicle is braking before a corner to match maximum corner speed
                case 'straight_brake'
                    state = 'straight brake';
                    brake_started = 1;
                    brake = 1;
                    max_torque_traction = 0;
                    acceleration(i+1) = braking(max_braking_force, brake);
                    motor_torque(i+1) = 0;
                    batt_current(i+1) = 0;
                    battery_SOC(i+1) = battery_SOC(i);
                    batt_ocv(i+1) = batt_ocv(i);
                    wheel_torque(i+1) = (acceleration(i+1) * mass) * tire_radius;
                    [velocity(i+1), distance_run(i+1), motor_RPM(i+1)] = velocity_distance(acceleration(i), velocity(i), distance_run(i), time_step, tire_radius, fdr, track_table, corner_count);
                    % vehicle is accelerating in a corner to get to max corner speed
                case 'corner_accel'
                    state = 'corner accel';
                    pedal = 1;
                    max_torque_traction(i) = corner_torque(velocity(i), lateral_g, mass, track_table, corner_count, tire_radius, fdr, driveline_eff, Cl, air_density, Al, acceleration(i), Cg_z, wheel_base);
                    [batt_current(i+1), motor_torque(i+1)] = motor( batt_ocv(i), batt_current(i), pack_R, motor_table_interp, motor_RPM(i), batt_current_limit, power_limit, max_torque_traction(i), max_motor_power, motor_eff, max_motor_current);
                    [battery_SOC(i+1), batt_ocv(i+1)] = battery(batt_current(i), battery_SOC(i), Ah, battery_table_interp, time_step, cell_s, batt_V_max);
                    wheel_torque(i+1) = driveline(motor_torque(i+1), fdr, driveline_eff);
                    [acceleration(i+1)] = acceleration_calc(wheel_torque(i+1), Cd, Af, Cr, air_density, mass, tire_radius, velocity(i));
                    [velocity(i+1), distance_run(i+1), motor_RPM(i+1)] = velocity_distance(acceleration(i), velocity(i), distance_run(i), time_step, tire_radius, fdr, track_table, corner_count);
                    [track_table_rows, ~] = size(track_table);
                    if (track_table_rows >= corner_count + 2) && (track_table(corner_count + 1, 1) == 1)
                        braking_distance = braking_distance_calc(velocity(i + 1), track_table(corner_count + 1, 4), max_braking_force);
                    end
                    % vehicle is at 0 acceleration and max speed for the corner
                case 'corner_constant'
                    state = 'corner constant';
                    [~, max_torque_traction(i)] = corner_speed(track_table(corner_count, 3), lateral_g, Cd, air_density, Af, Cr, mass, tire_radius, fdr);
                    [batt_current(i+1), motor_torque(i+1)] = motor( batt_ocv(i), batt_current(i), pack_R, motor_table_interp, motor_RPM(i), batt_current_limit, power_limit, max_torque_traction(i), max_motor_power, motor_eff, max_motor_current);
                    [battery_SOC(i+1), batt_ocv(i+1)] = battery(batt_current(i), battery_SOC(i), Ah, battery_table_interp, time_step, cell_s);
                    wheel_torque(i+1) = driveline(motor_torque(i+1), fdr, driveline_eff);
                    acceleration(i+1) = 0;
                    [velocity(i+1), distance_run(i+1), motor_RPM(i+1)] = velocity_distance(acceleration(i), velocity(i), distance_run(i), time_step, tire_radius, fdr, track_table, corner_count);
                    [track_table_rows, ~] = size(track_table);
                    if (track_table_rows >= corner_count + 2) && (track_table(corner_count + 1, 1) == 1)
                        braking_distance = braking_distance_calc(velocity(i + 1), track_table(corner_count + 1, 4), max_braking_force);
                    end
                case 'error'
                    fprintf('\nError Occurred, no state selected\n');
                    return;
                case 'done' %end lap
                    time = time_step * i;
                    
                    
                    if lap_count == 1
                        lap_time(lap_count) = time;
                        %fprintf('\nlap time: %d lap count: %d\n', lap_time(lap_count), lap_count)
                    else
                        lap_time(lap_count) = time - sum(lap_time);
                        %fprintf('\nlap time: %d lap count: %d\n', lap_time(lap_count), lap_count)
                    end
                    break;
                  
            end %end switch structure
            i = i + 1;
            section_distance = distance_run(i) - distance_run(start_i); %update distance traveled in a given section
            section_distances(i) = section_distance; %aidan add
            braking_distances(i) = braking_distance; %aidan add
            
            vehicle_state = state_selection(track_table, corner_count, section_distance, braking_distance, velocity, i, brake_started); 
            %update vehicle state inside a given corner count
        
        end %finish while loop
    end %finish one lap
end %finish all laps
i_count = i;

%%%Output Graphs%%%

% x = [0:time_step:time-time_step];
% y = [0:time_step:time];
% close all

% figure('Name' , 'Motor Table')
% plot(motor_table_interp(:,1), motor_table_interp(:,2));
% 
% figure('Name' , 'Velocity')
% plot(x, velocity);
% 
% figure('Name', 'Acceleration')
% plot(x, acceleration);
% 
% figure('Name', 'Motor Torque')
% plot(x, motor_torque);
% 
% figure('Name', 'Traction')
% plot(y, max_torque_traction);

fprintf('\nSimulation Completed\n');
        
                
