function truthExtractor = getTruthExtractor(truthFormat)
%getTruthExtractor function to get appropriate truth extractor function
%based on truthFormat.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

%By default 'Platform' and 'Actor' truth bus formats are supported. When
%truth format is selected as custom, a custom truth extractor function is
%expected in the 'TruthExtractorFcn' property.

if strcmpi(truthFormat,'Platform')    
    truthExtractor = @fusion.simulink.internal.platformExtractor;
elseif strcmpi(truthFormat,'Actor')
    truthExtractor = @fusion.simulink.internal.actorExtractor;
else
    coder.internal.assert(false,'fusion:simulink:OSPASimulinkBase:InvalidTruthFormat',truthFormat);
end
end