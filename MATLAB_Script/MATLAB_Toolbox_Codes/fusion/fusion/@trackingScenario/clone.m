function newScenario = clone(scenario)
%CLONE  Create a copy of a tracking scenario
%   newScenario = CLONE(Scenario) creates a copy of the trackingScenario,
%   Scenario.

%   Copyright 2019-2020 The MathWorks, Inc.

newScenario = reload(trackingScenario,scenario);
