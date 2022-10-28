classdef monostaticLidarSensor< lidarsim.internal.LidarSensor & radarfusion.internal.scenario.mixin.Sensor & scenario.internal.mixin.Perturbable
% MONOSTATICLIDARSENSOR Simulated lidar point cloud generator.
% SENSOR = MONOSTATICLIDARSENSOR(sensorIdx) returns a simulated lidar
% sensor to generate point clouds from scenes using default property
% values. The SensorIndex property is set to sensorIdx.
%
% SENSOR = MONOSTATICLIDARSENSOR(...,'Name',value) allows
% specifying properties of the sensor using name-value pairs.
% Unspecified properties have default values. See the list of
% properties below.
%
% MONOSTATICLIDARSENSOR properties:
%     SensorIndex               - Unique identifier of sensor
%     UpdateRate                - Sensor update rate (Hz)
%     MountingLocation          - Sensor's mounting location on platform (m)
%     MountingAngles            - Sensor's mounting angles on platform (deg)
%     MaxRange                  - Maximum detection range (m)
%     RangeAccuracy             - Accuracy of range measurements (m)
%     AzimuthResolution         - Azimuthal resolution of lidar (deg)
%     ElevationResolution       - Elevation resolution of lidar (deg)
%     AzimuthLimits             - Azimuth limits of the sensor (deg)
%     ElevationLimits           - Elevation limits of the sensor (deg)
%     HasNoise                  - Add noise to measurements
%     HasOrganizedOutput        - Output organized point cloud locations
%     HasINS                    - Enable input of platform's pose
%     DetectionCoordinates      - Coordinate system used to report detections
%
% MONOSTATICLIDARSENSOR methods:
%     step            - Generate point clouds from targets
%     perturbations   - Define perturbations to the monostaticLidarSensor
%     perturb         - Apply perturbations to the monostaticLidarSensor
%     release         - Allow property value and input characteristics changes
%     clone           - Create monostaticLidarSensor object with same property values
%     isLocked        - Locked status (logical)
%     <a href="matlab:help matlab.System/reset   ">reset</a>           - Reset states of monostaticLidarSensor object
%     <a href="matlab:help coverageConfig">coverageConfig</a>  - Report the monostaticLidarSensor object coverage configuration
% 
% Step method syntax:
%
% PTCLOUD = step(SENSOR, TGTMESHES, TIME) generates point cloud
% measurements from the 3-D geometry of targets, TGTMESHES, at the
% simulation time, TIME. TIME is a scalar value
%
% TGTMESHES is an array of struct. Each element of the array must
% contain the following fields.
%
%   PlatformID      A unique identifier for the target.
%
%   ClassID         A unique identifier for class of the target.
%
%   Position        A 3-element vector defininig the position of the
%                   target with respect to the frame of the sensor's
%                   mounting object.
%
%   Orientation     A quaternion object or a 3-by-3 orthonormal matrix
%                   defining the orientation of the target with respect
%                   to the frame of the sensor's mounting object.
%
%   Mesh            An extendedObjectMesh object representing the
%                   geometry of the target in its own coordinate frame.
%
% PTCLOUD is a N-by-3 or P-by-Q-by-3 matrix defining the locations of
% the point cloud. It is defined as a N-by-3 if HasOrganizedOutput
% property is set to false.
%
% The Coordinate system used to report locations of point cloud can be
% specified as:
%   "Sensor"       Locations are in sensor's Cartesian coordinates
%   "Body"         Locations are transformed to platform's Cartesian
%                  coordinates.
%   "Scenario":    Locations are transformed to scenario's Cartesian
%                  coordinates. 
%
%
% PTCLOUD = step(SENSOR, TGTMESHES, INSPOSE, TIME) allows
% providing the ins information about the sensor's platform. This
% syntax is enabled when HasINS property of the sensor is set to true.
% Using this syntax enable outputting locations of the point in PTCLOUD
% in the "Scenario" frame or to report INS information in the sensor
% configuration for tracking in scenario coordinates.
%
% INS must be a struct with the following fields:
%
%     Position       Position of sensor's platform estimated by INS in
%                    the scenario frame.
%
%     Orientation    Orientation of the sensor's platform estimated by
%                    INS in the scenario frame specified as
%                    a quaternion object or a 3-by-3 orthonormal
%                    matrix.
%
% [PTCLOUD, CONFIG] = step(SENSOR, ...) optionally
% returns the configuration of the sensor at current simulation time,
% CONFIG. The fields of the struct are defined <a href="matlab:help fusion.internal.interfaces.DataStructures/sensorConfigStruct">here</a>.
%
%
% [PTCLOUD, CONFIG, CLUSTERS] = step(...) optionally returns true
% cluster labels for each of the point in the point cloud, CLUSTERS.
% When HasOrganizedOutput is false, CLUSTERS is a N-by-2 vector
% defining the IDs of the target from which the point was generated.
% The first column of CLUSTERS represents the PlatformID of the target,
% which generated the point. The second column of CLUSTERS represents
% the ClassID of the target from which the point was generated. When
% HasOrganizedOutput if false, CLUSTERS is a P-by-Q-by-2 matrix, where
% first and second page of the matrix represents PlatformID and ClassID
% of the targets respectively.
%
% Example: Generate point cloud from a scenario
%  
%   % Define the scenario, ownship platform and a target
%   scenario = trackingScenario; 
%   ownship = platform(scenario); 
%   target = platform(scenario,'Trajectory',...
%                   kinematicTrajectory('Position',[10 -3 0],...
%                                       'Velocity',[5 0 0]));
%
%   % Specify the Mesh for the target to be a sphere. 
%   target.Mesh = extendedObjectMesh('sphere');
%
%   % Specify the dimensions of the target. The mesh will be scaled
%   % automatically
%   target.Dimensions.Length = 5; 
%   target.Dimensions.Width = 3; 
%   target.Dimensions.Height = 2;
%   
%   % Create a monostaticLidarSensor object with SensorIndex = 1 and
%   % UpdateRate = 10 Hz
%   sensor = monostaticLidarSensor('SensorIndex',1,'UpdateRate',10);
%   
%   % Advance scenario and generate the targets struct for the sensor
%   advance(scenario);
%   tgtmeshes = targetMeshes(ownship);
%    
%   % Step the sensor to generate the point cloud
%   time = scenario.SimulationTime;
%   [ptCloud, config, clusters] = sensor(tgtmeshes, time);
% 
%   % Visualize the point cloud
%   plot3(ptCloud(:,1),ptCloud(:,2),ptCloud(:,3),'.');
%
% See also:  <a href="matlab:help('fusion.scenario.Platform/targetMeshes')">targetMeshes</a>, extendedObjectMesh, lidarDetect

     
    % Copyright 2020-2021 The MathWorks, Inc.

    methods
        function out=monostaticLidarSensor
            % Assert that sensor index must be provided
        end

        function out=coverageConfig(~) %#ok<STOUT>
        end

        function out=defaultPerturbations(~) %#ok<STOUT>
        end

        function out=isAllowedInSystemBlock(~) %#ok<STOUT>
        end

        function out=loadObjectImpl(~) %#ok<STOUT>
            % Set properties in object obj to values in structure s
        end

        function out=saveObjectImpl(~) %#ok<STOUT>
            % Set properties in structure s to values in object obj
        end

        function out=stepImpl(~) %#ok<STOUT>
        end

    end
    properties
        pTimeLastQuery;

    end
end
