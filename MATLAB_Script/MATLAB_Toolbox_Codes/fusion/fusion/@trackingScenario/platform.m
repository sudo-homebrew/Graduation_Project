function plat = platform(scenario, varargin)
%platform  Create a new generic platform for a tracking scenario
%   plat = platform(scenario) adds a new platform to the trackingScenario,
%   scenario.
%
%   Platforms are objects that have:
%       * A trajectory defining how the platform's pose evolves over time
%       * Signatures defining the visibility of the platform to emitters and
%         sensors in the scenario
%   
%   Optionally, a platform may also include:
%       * A pose estimator to estimate the platform's current pose
%       * Emitters mounted on the platform which generate signals in the scenario
%       * Sensors mounted on the platform that detect other platforms or
%         the signals emitted from other platforms in the scenario
%
%   plat = platform(scenario, Name, Value, ...) specifies additional
%   name-value pair arguments described below:
%
%     ClassID           A user-defined integer intended to classify the
%                       type of platform.  The value of 0 is reserved for
%                       unknown classification (see objectDetection). The
%                       default value is 0.
%
%     Trajectory        A trajectory defining how the platform's pose
%                       changes as the scenario advances. By default, a
%                       kinematicTrajectory is used with default
%                       parameters. If the trackingScenario object's
%                       property IsEarthCentered is set to true, then by
%                       default a geoTrajectory is used.
%
%     Position          A position vector to be used as the Position of the
%                       default trajectory object. Position should only be
%                       specified when creating a stationary platform.
%                       Position and Trajectory cannot be specified
%                       jointly.
%
%     Orientation       Orientation defining the default orientation of the
%                       platform, specified as a quaternion, a rotation
%                       matrix, or a ZYX euler angles sequence. Orientation
%                       should only be specified when creating a stationary
%                       platform. For time-varying orientations, use a
%                       Trajectory. Orientation and Trajectory cannot be
%                       specified jointly.
%
%     Signatures        A cell array of irSignature, rcsSignature, and
%                       tsSignature objects or an empty cell array. The
%                       cell array contains at most only one instance for
%                       each type of the signature objects listed. A
%                       signature represents the reflection or emission
%                       pattern of a platform such as its radar
%                       cross-section, target strength, or IR intensity.
%                       The default value is {rcsSignature, irSignature,
%                       tsSignature}
%
%     Dimensions        A structure containing the length, width, height, 
%                       and origin offset of a cuboid that approximates the
%                       dimensions of the platform.   The origin offset is
%                       the offset from the geometric center of the cuboid
%                       where the position property of the platform's pose
%                       is located.  For example, an object whose position
%                       is centered at the rear face of the cuboid has 
%                       OriginOffset [-length/2, 0, 0].  
%                       The default is [0 0 0].
%
%     PoseEstimator     A pose estimator matching the interface defined by
%                       insSensor. By default, insSensor is used with all
%                       of its accuracy properties set to 0.
%
%     Emitters          A cell array of emitters mounted on the platform.
%                       By default, no emitters are mounted on the
%                       platform.
%
%     Sensors           A cell array of sensors mounted on the platform.
%                       By default, no sensors are mounted on the platform.
%
%   Upon creation, the platform will be given a unique integer id, which 
%   is assigned to the PlatformID property.
%
%   The 'SampleRate' property of the platform's trajectory will be set to
%   the 'UpdateRate' property of the trackingScenario, scenario. The
%   'SamplesPerFrame' property of the motion strategy will also be set to
%   1.
%
%   After creation, the following methods may be called on a platform:
%
%      <a href="matlab:help fusion.scenario.Platform/targetPoses">targetPoses</a> returns location and orientation for all other platforms relative to the platform
%      <a href="matlab:help fusion.scenario.Platform/pose">pose</a> returns the current pose estimate for the platform
%      <a href="matlab:help fusion.scenario.Platform/emit">emit</a> returns signals emitted by the emitters mounted on the platform
%      <a href="matlab:help fusion.scenario.Platform/detect">detect</a> returns detections from the sensors mounted on the platform
%
%   Example
%   -------
%   % Create a new scenario
%   scene = trackingScenario;
%
%   % Create a platform
%   plat = platform(scene);
% 
%   % Follow a circular trajectory 1 m in radius completing in one second.
%   plat.Trajectory = waypointTrajectory('Waypoints', [0 1 0; 1 0 0; 0 -1 0; -1 0 0; 0 1 0], ...
%                     'TimeOfArrival', [0; 0.25; .5; .75; 1.0]);
% 
%   % Perform the simulation
%   while scene.advance
%       p = pose(plat);
%       fprintf('Time = %f ', scene.SimulationTime);
%       fprintf('Position = [');
%       fprintf('%f ', p.Position);
%       fprintf('] Velocity = [');
%       fprintf('%f ', p.Velocity);
%       fprintf(']\n');
%   end
%
%   See also waypointTrajectory, rcsSignature, insSensor, radarEmitter, fusionRadarSensor.

%   Copyright 2017-2020 The MathWorks, Inc.

% auto-generate new ID based upon creation order

plat = platform@radarfusion.internal.scenario.Scenario(scenario, varargin{:});
