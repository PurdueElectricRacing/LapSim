config; %loads config file variables
fprintf('Simulation of %s initiated \n', vehicle_name)

%intialize variables
i = 1;
motor_RPM(i) = 0;
batt_ocv(i) = batt_V_max;
battery_SOC(i) = 100;
batt_current(i) = 0;
acceleration(i) = 0;
velocity(i) = 0;
distance(i) = 0;
%pre-process motor data
motor_table_interp = process_motor_table(motor_table);
battery_table_interp = process_battery_table(battery_table);
%run loop
pedal = 1;
while distance < 75
    [batt_current(i+1), motor_torque(i+1)] = motor(batt_ocv(i), batt_current(i), pack_R, motor_table_interp, pedal, motor_RPM(i), KV, kt, motor_eff, motor_R);
    [battery_SOC(i+1), batt_ocv(i+1)] = battery(batt_current(i), battery_SOC(i), Ah, battery_table_interp, time_step, cell_s);
    wheel_torque(i+1) = driveline(motor_torque(i+1), fdr, driveline_eff);
    [wheel_force(i), drag(i), rolling_resistance(i), acceleration(i+1)] = acceleration_calc(wheel_torque(i+1), Cd, Af, Cr, air_density, mass, tire_radius, velocity(i));
    [velocity(i+1), distance(i+1), motor_RPM(i+1)] = velocity_distance(acceleration(i), velocity(i), distance(i), time_step, tire_radius, fdr);
    i = i + 1;
end

time = (i-1) * time_step;
fprintf('\nLap Time: %s\n', time);