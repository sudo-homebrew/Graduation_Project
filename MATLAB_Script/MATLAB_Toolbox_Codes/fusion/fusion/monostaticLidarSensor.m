classdef monostaticLidarSensor < lidarsim.internal.LidarSensor & ...
        radarfusion.internal.scenario.mixin.Sensor & ...
        scenario.internal.mixin.Perturbable
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
    %     reset           - Reset states of monostaticLidarSensor object
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
    
    properties (Access = protected)
        pTimeLastQuery = 0;
    end
    
    methods
        function obj = monostaticLidarSensor(varargin)
            % Assert that sensor index must be provided
            [args, sensorIndex] = parseSensorIndex(varargin{:});
            
            % Use default HasOrganizedOutput as false.
            obj@lidarsim.internal.LidarSensor('HasOrganizedOutput',false,args{:});
            obj.SensorIndex = sensorIndex;
            obj.OutputFormat = 'matrix';
        end
    end
    
    methods (Access = protected)
        function [ptCloud, config, clusters] = stepImpl(obj, varargin)
            [ptCloud, configBase, clusters] = stepImpl@lidarsim.internal.LidarSensor(obj,varargin{:});
            % Adapt config with other sensors
            config = obj.sampleConfiguration;
            config.SensorIndex = configBase.SensorIndex;
            config.IsValidTime = configBase.IsValidTime;
            config.IsScanDone = true; % No scanning
            config.FieldOfView = configBase.SensorLimits(1:2,:);
            config.RangeLimits = [0 obj.MaxRange];
            config.MeasurementParameters = configBase.SensorTransformParameters;
        end

        function loadObjectImpl(obj,s,wasLocked)
            % Set properties in object obj to values in structure s
            
            % Set perturbation related properties
            loadPerts(obj,s);
            
            % Set private and protected properties
            if isfield(s, 'pTimeLastQuery')
                obj.pTimeLastQuery = s.pTimeLastQuery;
            end
            
            % Set public properties and states
            loadObjectImpl@lidarsim.internal.LidarSensor(obj,s,wasLocked);
        end

        function s = saveObjectImpl(obj)
            % Set properties in structure s to values in object obj

            % Set public properties and states
            s = saveObjectImpl@lidarsim.internal.LidarSensor(obj);
            
            % Set private and protected properties
            s.pTimeLastQuery = obj.pTimeLastQuery;
            
            % Set perturbation related properties
            s = savePerts(obj, s);
        end
    end
    
    
    % Methods for interacting with platform
    methods (Access = {?matlab.unittest.TestCase,?radarfusion.internal.scenario.Platform,?radarfusion.internal.scenario.mixin.SampledDevice})
        function newTime = nextValidUpdateTime(obj, currentTime)
            % Returns the next admissible time by the sensor
            % if the sensor has not yet started, return currentTime.
            if obj.pHasFirstUpdate
                elapsedInterval = currentTime-obj.pTimeLastQuery;
                rate = obj.UpdateRate;
                numInts = ceil((elapsedInterval+obj.pSmallValue)*rate);
                newTime = obj.pTimeLastQuery + numInts/rate;
            else
                newTime = currentTime;
            end
            obj.pTimeLastQuery = newTime;
        end
    end
    
    methods (Hidden)
        function tf = requiresSignals(~)
            tf = false;
        end
        function tf = requiresConfig(~)
            tf = false;
        end
        function tf = supportsSignals(~)
            tf = false;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase,?fusion.scenario.Platform})
        function config = sampleConfiguration
            params = fusion.internal.interfaces.DataStructures.measurementParametersStruct;
            config = struct('SensorIndex',1,...
                'IsValidTime',false,...
                'IsScanDone',false,...
                'FieldOfView',zeros(2,2),...
                'RangeLimits',[0 inf], ...
                'RangeRateLimits',[0 inf], ...
                'MeasurementParameters',params);
        end
    end
    
    methods (Access = protected)
        function perts = defaultPerturbations(~)
            perturbableProps = {"RangeAccuracy";"AzimuthResolution";"ElevationResolution"}; %#ok<CLARRSTR>
            perts = struct('Property',perturbableProps,...
                'Type',"None",...
                'Value',{{NaN NaN}}...
                );
        end
    end
    
    methods
        function config = coverageConfig(obj, position, orientation)
            narginchk(1,3);
            if nargin < 2
                position = zeros(1,3);
            end
            if nargin < 3
                orientation = quaternion.ones();
            end
            
            scene2plat = rotmat(orientation,'frame');
            pos = position(:) + scene2plat'*obj.MountingLocation(:);
            
            plat2sens = quaternion(obj.MountingAngles,'eulerd','ZYX','frame');
            orient = orientation*plat2sens;

            config.Index = obj.SensorIndex;
            config.LookAngle = [0;0];
            
            config.FieldOfView = [obj.AzimuthLimits;obj.ElevationLimits];
            config.ScanLimits = [obj.AzimuthLimits;obj.ElevationLimits];
            config.Range = obj.MaxRange;
            config.Position = pos';
            config.Orientation = orient;
        end
    end
    
    methods (Static)
        function tf = isAllowedInSystemBlock
            tf = false;
        end
    end
end

function [args, sensorIndex] = parseSensorIndex(varargin)
% Don't enforce constructor inputs when the object is loading
isLoading = fusion.internal.remotesensors.ScanningMount.isLoading;
if isLoading
    args = varargin;
    sensorIndex = 1;
    return;
end

senIndexID = fusion.internal.findProp('SensorIndex',varargin{:});
foundInPVPairs = senIndexID <= numel(varargin) - 1;
foundAtFirstArg = mod(nargin,2) == 1 && isnumeric(varargin{1});
coder.internal.assert(foundInPVPairs || foundAtFirstArg,...
    'shared_radarfusion:RemoteSensors:mustSpecifyProperty','SensorIndex');

if foundInPVPairs && foundAtFirstArg
    % Found at both places
    sensorIndex = varargin{senIndexID + 1};
    args = {varargin{2:senIndexID-1},varargin{senIndexID+2:end}};
elseif foundInPVPairs
    % Found only in PV pairs
    sensorIndex = varargin{senIndexID + 1};
    args = {varargin{1:senIndexID-1},varargin{senIndexID+2:end}};
else
    % Found at first arg
    sensorIndex = varargin{1};
    args = varargin(2:end);
end
end

