function configs = coverageConfig(obj)
%COVERAGECONFIG returns configurations used by a coverage plotter.
%   configs = coverageConfig(scenario) returns an array of sensor and
%   emitter configuration structures from all sensors and emitters in the
%   tracking scenario.  
%
%   The fields of the structure are listed <a href="matlab:help fusion.internal.interfaces.DataStructures/coverageConfigStruct">here</a>.
%
%   See also trackingScenario.

%   Copyright 2020 The MathWorks, Inc.

configs = coverageConfig@radarfusion.internal.scenario.Scenario(obj);