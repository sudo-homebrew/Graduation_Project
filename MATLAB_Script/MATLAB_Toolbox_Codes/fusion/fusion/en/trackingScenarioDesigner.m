%trackingScenarioDesigner Design tracking scenarios, configure platforms and sensors, and
%generate synthetic object detections
%
%   trackingScenarioDesigner opens the Tracking Scenario Designer app for
%   designing trackingScenario objects, configuring platforms and
%   trajectories, configuring scanning monostatic radar sensors, and
%   generating synthetic sensor data.
%
%   trackingScenarioDesigner(sessionFileName) opens the app and loads the
%   specified scenario file that was previously saved from the app.
%
%   trackingScenarioDesigner(scenario) opens the app and loads the specified
%   trackingScenario object into the app with the following limitations:
%   * The scenario must have the IsEarthCentered property set to false.
%   * The scenario's StopTime and UpdateRate properties are ignored.
%   * Platform ClassID properties must correspond to the defaults in the app.
%   * Platform PlatformID properties are renumbered in numeric sequence.
%   * Only monostaticRadar sensors are supported, others are removed.
%   * Emitters are removed from the scenario.
%
%   See also trackingScenario, fusionRadarSensor

 
% Copyright 2020 The MathWorks, Inc.

