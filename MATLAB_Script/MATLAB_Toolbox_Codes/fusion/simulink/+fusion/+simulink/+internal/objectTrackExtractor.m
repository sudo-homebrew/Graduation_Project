function tracks = objectTrackExtractor(x)
%OBJECTTTRACKEXTRACTOR function to extract the track input from a object track Simulink bus.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

tracks =  x.Tracks(1:x.NumTracks);
end