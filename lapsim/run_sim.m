%%%% run_sim.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script parses the configuration file,         %
% sets up classes, and runs the lap simulator with   %
% the GUI (coming soon)                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Housekeeping
close all                                               % Close all figures
warning('OFF', 'MATLAB:table:ModifiedAndSavedVarnames') % Turn off stupid warning
clc                                                     % Clear the terminal
clear all                                               % Disregard error. It's important to clear persistent vars

%% Value Loading
cfg_f = 'init_cfg.csv';                                 % Configuration file name
tk_f = 'autox.csv';                                     % Track file name
current_rev = 1.0;                                      % This number must match the configuration file's revision number
dt = 0.01;                                              % Timestep for the lapsim
sim_time = 0;                                           % Current point in time for the lapsim

% Read the config file and return constructed classes
[ae, bat, co, dlne, dr, h, l, mo, v, su] = load_cfg(cfg_f, current_rev);

tk = trk(tk_f);                                         % Read the track file and return the constructed track class

% if strcmp(cfg_f, 'init_cfg.csv')                        % Check if someone attempted to use the init cfg
%     fprintf('Error: Attempted to use initial config\n') % Yes, so let the user know
%     fprintf('Please update and rename the file\n')
%     return;
% end

fprintf('Loaded configuration file: %s\n', cfg_f)       % Let the user know which config they loaded in case they forgot to change it
fprintf('Loaded track file: %s\n', tk_f)                % Let the user know which track they loaded in case they forgot to change it

%% Precomputation
max_table = v.max_corner_speed(tk.raw_track);           % Compute the max speed for each corner

%% Main Run Loop
i = 1;
while v.k < tk.elements
    v.vd_main(ae, dlne, mo, tk.raw_track, max_table, dt);
    ae.aero_calc(v);
    l.powerLoss(dt);
    l.updateCooling();
    mo.motor_run(bat, v, dlne);
%    [t,x] = bat.runcircuit(mo, l, h, sim_time, dt);
%     v.update_position(sim_time); % Currently does nothing
    sim_time = sim_time + dt;
    pd(i) = mo.power_draw;
    if pd(i) == -1
        mt(i) = mo.motor_torque;
        ms(i) = mo.motor_speed;
    end
    pdlv(i) = l.lv_power_cons;
    tme(i) = sim_time - dt;
    i = i + 1;
end

%% Post Processing
fprintf("Max speed: %f\n", v.max_velocity)
fprintf("Total time: %f\n", sim_time - dt)
