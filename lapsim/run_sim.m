%%%% run_sim.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script parses the configuration file,         %
% sets up classes, and runs the lap simulator with   %
% the GUI (coming soon)                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Housekeeping
close all                                               % Close all figures
warning('OFF', 'MATLAB:table:ModifiedAndSavedVarnames') % Turn off stupid warning
clc                                                     % Clear the terminal

%% Value Loading
cfg_f = 'init_cfg.csv';                                 % Configuration file name
tk_f = 'accel.csv';                                     % Track file name
current_rev = 1.0;                                      % This number must match the configuration file's revision number

% Read the config file and return constructed classes
[ae, bat, co, dlne, dr, h, l, mo] = load_cfg(cfg_f, current_rev);

tk = trk(tk_f);                                         % Read the track file and return the constructed track class

if strcmp(cfg_f, 'init_cfg.csv')                        % Check if someone attempted to use the init cfg
    fprintf('Error: Attempted to use initial config\n') % Yes, so let the user know
    fprintf('Please update and rename the file\n')
    return;
end

fprintf('Loaded configuration file: %s\n', cfg_f)       % Let the user know which config they loaded in case they forgot to change it
fprintf('Loaded track file: %s\n', trk_f)               % Let the user know which track they loaded in case they forgot to change it

%% Precomputation
max_table = max_corner_speed(track_table);              % Compute the max speed for each corner

%% Main Run Loop
for i = 1:tk.elements
    % VD Calculations
    vd_main(ae, dlne);
    [fr, fl, rr, rl] = weight_transfer_variable(self,rho);
    update_position(self, sim_time);
    [dfr, dfl, drr, drl] = weight_transfer(self);
    calculate_accel(self);
    
    % Aero Calculations
    aero_calc(self, velocity)
    
    % Battery Calculations
    total_cap = get.total_cap(obj);
    power_out = get.power_out(obj);
    [motor, battery, DCDC,time] = runcircuit(motor, battery, cables, DCDC, time, timestep);
    
    % Driveline Calculations
    [wheel_torque_left, wheel_torque_right] = RWD_Driveline(self);
    difTorque = differential(RAV,LAV);
    wheel_torque = HM_Driveline(self);
    
    % LV Calculations
    powerLoss(self, dt);
    updateCooling(self);
    
    % Motor Calculations
    motor_run(self, battery, vd, max_torque_traction);
    motor_corner(self, battery, vd, fdr, motor_table_interp);
    motor_corner_accel(self, battery, motor_table_interp, driver);
    
    % Suspension Calculations
    shock_force(self);
    
    % Tire Calculations
    update_tire(self, Fz, steering_angle, veh_vel_vector, angular_vel);
end

%% Post Processing