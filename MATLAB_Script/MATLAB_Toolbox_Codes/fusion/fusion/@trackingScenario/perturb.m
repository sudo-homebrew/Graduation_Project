function varargout = perturb(scenario)
%PERTURB  Perturb the scenario
%  PERTURB(SCENARIO) perturbs the baseline trackingScenario, SCENARIO. Each
%  object property in the scenario is perturbed according the perturbations
%  defined for it. Object properties that have no perturbations defined are
%  not modified.
%
%  OFFSETS = PERTURB(SCENARIO) returns the offset values from the baseline
%  scenario.
%
% See also: trackingScenario, platform, waypointTrajectory, insSensor

%   Copyright 2019-2020 The MathWorks, Inc.

[varargout{1:nargout}] = perturb@radarfusion.internal.scenario.Scenario(scenario);
