function [detections, configurations, sensorPIDs] = detect(obj, varargin)
%DETECT Collect detections from all the sensors in the scenario
%    detections = DETECT(s) reports the detections from all sensors mounted
%    on every platform in the trackingScenario, s, that do not process
%    emissions returned by emit(s).
%  
%    detections = DETECT(s, signals) reports the detections from the
%    sensors that process emissions without information about emitter
%    configurations (i.e. passive and bistatic sensors).  
%  
%    detections = DETECT(s, signals, configs) reports the detections from
%    all sensors in the scenario, including those that require information
%    about emitter configurations (i.e. some monostatic sensor model
%    configurations such as those that model cross-talk between sensors.)
%  
%    Detections are always returned as a cell array of
%    objectDetection objects.
%  
%    [...,configs] = DETECT(...), additionally, returns a struct
%    array of the configurations of each sensor at the detection
%    time.
%
%    [...,sensorConfigPIDs] = DETECT(...), additionally, returns a column
%    vector of all platform IDs that correspond to the configs.
%
%   % Example: Obtain the detections from 2 platforms in the scenario
%   s = rng(0); % For repeatable result
%   ts = trackingScenario('UpdateRate', 1);
%   plat1 = platform(ts);
%   plat1.Trajectory.Position = [0,0,0];
%   emitter1 = radarEmitter(1, 'UpdateRate', 1);
%   sensor1 = fusionRadarSensor('SensorIndex',1,'DetectionMode','Monostatic','EmitterIndex',1,'RangeResolution',1,'EmissionsInputPort',true);
%   plat1.Emitters = emitter1;
%   plat1.Sensors = sensor1;
%   plat2 = platform(ts);
%   plat2.Trajectory.Position = [100,0,0];
%   emitter2 = radarEmitter(2, 'UpdateRate', 1);
%   sensor2 = fusionRadarSensor('SensorIndex',2,'DetectionMode','Monostatic','EmitterIndex',2,'RangeResolution',1,'EmissionsInputPort',true);
%   plat2.Emitters = emitter2;
%   plat2.Sensors = sensor2;
%   
%   advance(ts);
%   [emtx, emitterConfs, emitterConfPIDs] = emit(ts); % Transmitted emissions
%   emprop = propagate(ts, emtx, 'HasOcclusion', true); % Propagate emissions
%   [dets, sensorConfs, sensorConfPIDs] = DETECT(ts, emprop, emitterConfs);
%
%   % Platform 1 detects platform 2:
%   disp(dets{1})
%
%   % Return the random number generator to its previous state
%   rng(s)
%
%   See also trackingScenario, <a href="matlab:help('fusion.scenario.Platform/detect')">detect</a>, emit, propagate.

%   Copyright 2019-2020 The MathWorks, Inc.

[detections, configurations, sensorPIDs] = detect@radarfusion.internal.scenario.Scenario(obj, varargin{:});
