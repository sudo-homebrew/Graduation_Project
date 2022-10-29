function trackExtractor = getTrackExtractor(trackFormat)
%getTrackExtractor function to get appropriate track extractor function
%based on trackFormat.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

%By default 'objectTrack' track bus formats is supported. When track format
%is selected as custom, a custom track extractor function is
%expected in the 'TrackExtractorFcn' property.

if strcmpi(trackFormat,'objectTrack')
    trackExtractor = @fusion.simulink.internal.objectTrackExtractor;
else
    coder.internal.assert(false,'fusion:simulink:OSPASimulinkBase:InvalidTrackFormat',trackFormat);
end
end