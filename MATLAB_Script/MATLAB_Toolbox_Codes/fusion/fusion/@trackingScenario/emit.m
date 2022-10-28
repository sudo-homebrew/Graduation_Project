function [emissions, configurations, sensorPIDs] = emit(obj)
%EMIT Collect emissions from emitters in the scenario
%   emissions = EMIT(s) reports the signal emitted from all the emitters
%   mounted on platforms in the scenario, s.
%  
%   Emissions are always returned as a cell array of emissions from radar
%   or sonar.
%  
%   [...,configs] = EMIT(...), additionally, returns a struct array of
%   the configurations of each emitter at the emission time.
%
%   [...,sensorConfigPIDs] = EMIT(...), additionally, returns a column
%   vector of all platform IDs that correspond to the configs.
%
%   % Example: Obtain the emissions from two platforms in the scenario
%   ts = trackingScenario('UpdateRate', 1);
%   plat1 = platform(ts);
%   plat1.Trajectory.Position = [0,0,0];
%   emitter1 = radarEmitter(1, 'UpdateRate', 1);
%   plat1.Emitters = emitter1;
%   plat2 = platform(ts);
%   plat2.Trajectory.Position = [100,0,0];
%   emitter2 = radarEmitter(2, 'UpdateRate', 1);
%   plat2.Emitters = emitter2;
%   
%   advance(ts);
%   [emissions, configs, sensorConfigPIDs] = EMIT(ts);
%
%   % There should be 2 emissions, one from each emitter
%   disp("There are " + numel(emissions) + " emissions."); 
%   disp("The first emission is:")
%   disp(emissions{1});
%   disp("The second emission is:")
%   disp(emissions{2});
%   disp("The emitter configuration associated with the first emission is:");
%   disp(configs(1));
%   disp("The emitter configuration associated with the second emission is:");
%   disp(configs(2));
%   disp("The emitter configurations are connected with platform IDs: ");
%   disp(sensorConfigPIDs');
%
%   See also trackingScenario, <a href="matlab:help('fusion.scenario.Platform/emit')">emit</a>, propagate, detect.

%   Copyright 2019-2020 The MathWorks, Inc.

[emissions, configurations, sensorPIDs] = emit@radarfusion.internal.scenario.Scenario(obj);
