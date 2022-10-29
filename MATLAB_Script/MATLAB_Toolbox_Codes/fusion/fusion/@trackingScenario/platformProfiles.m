function profiles = platformProfiles(scene)
%platformProfiles Return profiles of tracking scenario platforms
%   platformProfiles(scene) returns a structure of all platforms in the
%   scenario, scene, with the following fields:
%
%      PlatformID         Unique integer identifier specific to this platform
%
%      ClassID            User-defined integer identifier for the
%                         classification ID
%
%      Dimensions         A structure containing the length, width, height, 
%                         and origin offset of a cuboid that approximates the
%                         dimensions of the platform.   The origin offset is
%                         the offset from the geometric center of the cuboid
%                         where the position property of the platform's pose
%                         is located.  For example, an object whose position
%                         is centered at the rear face of the cuboid has 
%                         OriginOffset [-length/2, 0, 0].  
%                         The default is [0 0 0].
%
%      Signatures         Cell array of signatures defining visibility of
%                         this platform to emitters and sensors in the
%                         scenario
%
%   Example
%   -------
%   % Create a scenario with two platforms and retrieve the radar cross
%   % section (RCS) signatures of each platform.
%
%   % Create a new scenario
%   scene = trackingScenario
%
%   % Add two platforms with different RCS signatures
%   p1 = platform(scene)
%   p2 = platform(scene)
%
%   profiles = platformProfiles(scene);
%
%   % Show information for the first platform
%   profiles(1)
%
%   % Show information for the second platform
%   profiles(2)
%
%   See also platformPoses.

%   Copyright 2018-2020 The MathWorks, Inc.

profiles = platformProfiles@radarfusion.internal.scenario.Scenario(scene);
