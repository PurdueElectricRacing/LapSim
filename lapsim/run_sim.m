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

%% Main Run Loop
for i = 1:tk.elements
    
end

%% Post Processing