%%%% run_sim.m %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script parses the configuration file,         %
% sets up classes, and runs the lap simulator with   %
% the GUI                                            %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
current_rev = 1.0;                                  % This number must match the configuration file's revision number
classes = load_cfg('init_cfg.csv', current_rev);    % Read the config file and return constructed classes

