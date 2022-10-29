function truths = platformExtractor(x)
%PLATFORMEXTRACTOR function to extract the truth input from a Platform Simulink bus.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

truths = x.Platforms(1:x.NumPlatforms);
end