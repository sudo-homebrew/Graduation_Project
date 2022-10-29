function [rout, currRNG, prevRNG] = record(scene, varargin)
%RECORD Record simulation.
%   r = RECORD(scene) starts the simulation from the beginning and stores
%   the results into the record until the scenario's StopTime is reached or
%   any platform has finished following the trajectory specified by
%   the platform's 'Trajectory' property.
%
%   The record, r, can be a trackingScenarioRecording or an array of struct
%   with the following fields:
%
%       SimulationTime    the time of simulation
%       Poses             a struct array containing the pose of each 
%                         platform at the corresponding simulation time.
%
%   r = RECORD(scene, fmt) specifies the pose orientation format as one of
%   the following:
%      'quaternion' (default) - returns orientation as a quaternion.
%      'rotmat'               - returns orientation as a rotation matrix.
%
%   r = RECORD(..., 'Name', value) allows you to add to the recording by
%   specifying the following name-value pairs:
%      'IncludeEmitters'  a flag. If true, records the fields Emissions,
%                         EmitterConfigurations, and EmitterPlatformIDs.
%                         Default: false.
%      'IncludeSensors'   a flag. If true, records the fields Detections,
%                         SensorConfigurations, and SensorPlatformIDs.
%                         Default: false.
%      'InitialSeed'      allows you to specify the initial random seed for
%                         the recording. If specified, the random number
%                         generator will be set to this seed and 'Twister'
%                         before the recording and reset to the current rng
%                         at the end of the recording.
%                         Default: use current random seed.
%      'HasOcclusion'     a flag, If true, propagated radar emissions will
%                         account for occlusions in the scene.
%                         Default: true.
%      'RecordingFormat'  Choose the recording output format:
%                           'Struct'     An array of struct (Default)
%                           'Recording'  A trackingScenarioRecording object
%      'CoordinateSystem' Choose the coordinate system and axes used to
%                         express platform poses for Earth-centered
%                         scenarios. This property can only be specified if
%                         scene.IsEarthCentered is true:
%                           'Cartesian' (Default)
%                           'Geodetic'   
%                         See <a href="matlab:help trackingScenario/platformPoses">platformPoses</a> for additional information.
%
%  The additional fields controlled by the flag 'IncludeEmitters' are:
%      Emissions              a cell array of emissions in the scene
%      EmitterConfigurations  a struct array of emitter configurations
%      EmitterPlatformIDs     a numeric array of platform IDs for each emitter
%      CoverageConfig         a struct array of emitter coverage configurations
%
%  The additional fields controlled by the flag 'IncludeSensors' are:
%      Detections             a cell array of objectDetection objects made 
%                             by the sensors in the scene
%      SensorConfigurations   a struct array of sensor configurations 
%      SensorPlatformIDs      a numeric array of platform IDs for each sensor
%      CoverageConfig         a struct array of sensor coverage configurations
%
%   Example 1: Record pose information
%   ----------------------------------
%   % Create a new scenario
%   scene = trackingScenario
%   
%   % add a platform
%   plat = platform(scene)
%   
%   % tell it to follow a 25 m trajectory along the x-axis at 20 m/s.
%   plat.Trajectory = waypointTrajectory('Waypoints',[0 0 0; 25 0 0], ...
%                      'TimeOfArrival', [0 25/20]);
%   
%   % run simulation, storing results
%   r = RECORD(scene)
%    
%   % show first result
%   r(1)
%   r(1).Poses
%
%   % show last result
%   r(end)
%   r(end).Poses
%
%   Example 2: Record pose, emissions, and detections
%   -------------------------------------------------
%   % Load a scenario
%   load ATCScenario scenario
%   
%   % run simulation, storing results
%   r = RECORD(scenario, 'quaternion', 'IncludeEmitters', true,...
%      'IncludeSensors', true, 'InitialSeed', 2019)
%    
%   % show first result
%   r(1)
%
%   % show last result
%   r(end)
%
%   See also restart, advance, platformPoses, trackingScenarioRecording,
%   coverageConfig

%   Copyright 2017-2020 The MathWorks, Inc.

[rout, currRNG, prevRNG] = record@radarfusion.internal.scenario.Scenario(scene, varargin{:});

