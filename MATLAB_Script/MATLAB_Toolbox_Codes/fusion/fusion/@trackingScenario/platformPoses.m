function poses = platformPoses(scene, varargin)
%platformPoses Return location and orientation of tracking scenario platforms.
%   poses = platformPoses(scene) returns the location and orientation of
%   all platforms in the scenario, scene, at the current simulation time.
%   The results are returned in an array of platform pose structures,
%   poses. The fields of the returned structures are listed <a href="matlab:help fusion.internal.interfaces.DataStructures/platformPoseStruct">here</a>.
%
%   poses = platformPoses(scene,fmt) returns the pose orientations using
%   one of the following formats:
%      'Quaternion' (default) - returns orientation as a quaternion.
%      'Rotmat'               - returns orientation as a rotation matrix.
%
%   poses = platformPoses(... ,'CoordinateSystem',coord) allows you to
%   specify the choice of coordinate system and axes. This optional syntax
%   is only used when scene.IsEarthCentered is true. Accepted values for
%   coord are:
%      'Cartesian'  (default) - returns quantities using Cartesian
%                               coordinates in the
%                               Earth-Centered-Earth-Fixed axes
%      'Geodetic'             - returns position using geodetic coordinates
%                               (latitude, longitude, and altitude),
%                               orientation, velocity, and acceleration are
%                               expressed in the local reference axes of
%                               each platform (North-East-Down by default).
%
%   Example
%   -------
%   % Create a scenario with a platform and retrieve its position and
%   % orientation information.
%
%   % Create a new scenario
%   scene = trackingScenario
%
%   % Add a platform and extract its pose information
%   plat = platform(scene);
%   plat.Trajectory.Position = [1 1 0];
%   plat.Trajectory.Orientation = quaternion([90 0 0], 'eulerd', 'ZYX', 'frame');
%   poses = platformPoses(scene)
%
%   % Now extract the orientation information in matrix format
%   poses = platformPoses(scene,'rotmat');
%   poses.Orientation
%
%
%   See also platformProfiles, platform.

%   Copyright 2017-2020 The MathWorks, Inc.

poses = platformPoses@radarfusion.internal.scenario.Scenario(scene, varargin{:});
