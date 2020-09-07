classdef trk < handle
    properties
        raw_track                                               % Radius of element (0 for straights)
        elements                                                % The number of individual track elements
    end
    
    methods
        function obj = trk(filename)
            values = readtable(filename);                       % Load the track file into Matlab
            obj.raw_track = table2array(values(2:end, 1:end));  % Pull out only valid cells
            [rows, ~] = size(obj.raw_track);                    % Save number of elements for run loop
            obj.elements = rows;                                % Save number of elements for run loop
        end
    end
end

