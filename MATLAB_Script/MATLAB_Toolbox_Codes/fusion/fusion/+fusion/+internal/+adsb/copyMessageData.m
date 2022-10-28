function mess = copyMessageData(mess, other)
%copyMessageData Copy ADS-B message information over
% out = copyMessageData(in, other) reads the data from the other struct and
% copies it to the in struct. The result is output as out
%
% This is an internal function and may be removed or modified in a future
% release.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

%Go through all fields
fnames = fieldnames(other);
for i=1:numel(fnames) 
    f = fnames{i};
    val = other.(f);
    % Update message
    switch f
        case 'ICAO'
            mess.(f) = char(val);
        case 'Time'
            if ~isnan(val)
                mess.(f) = val;
            end
        case 'Category'
            if val~=0
                mess.(f) = val;
            end
        case 'Callsign'
            if ~isempty(val)
                mess.(f) = val;
            end
        case 'Latitude'
            if ~isnan(val)
                mess.(f) = val;
            end
        case 'Longitude'
            if ~isnan(val)
                mess.(f) = val;
            end
        case 'Altitude'
            if ~isnan(val)
                mess.(f) = val;
            end
        case 'Veast'
            if ~isnan(val)
                mess.(f) = val;
            end
        case 'Vnorth'
            if ~isnan(val)
                mess.(f) = val;
            end
        case 'ClimbRate'
            if ~isnan(val)
                mess.(f) = val;
            end
        case 'Heading'
            if ~isnan(val)
                mess.(f) = val;
            end
        case 'NACPosition'
            if val~=0
                mess.(f) = val;
            end
        case 'GeometricVerticalAccuracy'
            if val ~= 0
                mess.(f) = val;
            end
        case 'NACVelocity'
            if val~=0
                mess.(f) = val;
            end
        otherwise
            %
    end
end
end
