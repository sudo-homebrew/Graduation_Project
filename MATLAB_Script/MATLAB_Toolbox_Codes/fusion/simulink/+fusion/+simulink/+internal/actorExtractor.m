function truths = actorExtractor(x)
%PLATFORMEXTRACTOR function to extract the truth input from an Actor Simulink bus.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2020 The MathWorks, Inc.

%#codegen

truths = x.Actors(1:x.NumActors);
end

