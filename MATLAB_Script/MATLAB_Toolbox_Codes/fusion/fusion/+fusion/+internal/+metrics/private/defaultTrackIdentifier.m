function trackIDs = defaultTrackIdentifier(x)
%DEFAULTTRACKIDENTIFIER default track identifier implementation
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

if isempty(x)
    trackIDs = nan(0,1);
else
    if coder.target('MATLAB')
        trackIDs = vertcat(x.TrackID); 
    else
        trackIDs = zeros(numel(x),1); % Always double, cast it to right data type in the caller.
        for i = 1:numel(x)
            trackIDs(i) = x(i).TrackID;
        end
    end
end
