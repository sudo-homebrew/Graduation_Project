classdef Platform < radarfusion.internal.scenario.Platform
    %fusion.scenario.Platform create platform for fusionScenario
    %
    %   fusion.scenario.Platform contains properties and methods that govern
    %   the motion, pose, size, and signatures of a platform object in a fusion
    %   scenario.
    %
    %   See also trackingScenario/platform.
    
    %   Copyright 2017-2020 The MathWorks, Inc.
    
    % required properties
    properties
        %Signatures  Cell array of signature objects
        % A cell array specified as a cell array of irSignature,
        % rcsSignature, and tsSignature objects or an empty cell array. The
        % cell array contains at most only one instance for each type of
        % the signature objects listed. A signature represents the
        % reflection or emission pattern of a platform such as its radar
        % cross-section, target strength, or IR intensity.
        %
        % Default: {rcsSignature, irSignature, tsSignature}
        Signatures
    end
    
    % extended properties
    properties
        %Mesh Object mesh
        % An object of type extendedObjectMesh, which represents the
        % geometry of the platform using face, vertex information. This
        % information is used by monostaticLidarSensor to generate point
        % cloud data.
        %
        % Default: cuboid mesh
        Mesh
    end
    
    % Setters and getters
    methods
        function set.Signatures(obj,val)
            val = radarfusion.internal.scenario.Platform.mkcell(val);
            radarfusion.internal.scenario.Platform.validateSignatures(val);
            obj.Signatures = val;
        end
        
       function set.Mesh(obj, val)
            fusion.scenario.Platform.validateMesh(val);
            obj.Mesh = val;
        end
    end
    
    % Constructor
    methods
        function obj = Platform(scenario, id, varargin)
            obj@radarfusion.internal.scenario.Platform(scenario, id, varargin{:});
        end
    end
    
    % Public methods
    methods
        function targetsOut = targetPoses(obj, varargin)
            %targetPoses Return location and orientation for all other platforms
            %   poses = targetPoses(plat) returns the location and orientation of all
            %   other platforms in the platform's body frame.  The platform, plat, must
            %   be previously added to the tracking scenario via the platform method.
            %   plat is returned by calling the platform method of tracking scenario or
            %   can also be retrieved from the Platform property of a tracking
            %   scenario. The results are returned in an array of platform pose
            %   structures, poses. The fields of the returned structures are listed
            %   <a href="matlab:help fusion.internal.interfaces.DataStructures/platformPoseStruct">here</a>.
            %
            %   poses = targetPoses(plat,fmt) returns the pose orientations using
            %   one of the following formats:
            %      'quaternion' (default) - returns orientation as a quaternion.
            %      'rotmat'               - returns orientation as a rotation matrix.
            %
            %   Example
            %   -------
            %   scene = trackingScenario('UpdateRate',10);
            %
            %   % add a few platforms
            %   plat1 = platform(scene)
            %   plat2 = platform(scene)
            %
            %   % Tell the first platform to follow a circular trajectory 1 m in radius
            %   % completing in one second.
            %   plat1.Trajectory = waypointTrajectory('Waypoints', [0 1 0; 1 0 0; 0 -1 0; -1 0 0; 0 1 0], ...
            %                  'TimeOfArrival', [0; 0.25; .5; .75; 1.0]);
            %
            %   % Tell the second platform to follow a circular trajectory 1 m in
            %   %  radius completing in 0.8 seconds.
            %   plat2.Trajectory = waypointTrajectory('Waypoints', [0 1 0; 1 0 0; 0 -1 0; -1 0 0; 0 1 0], ...
            %                  'TimeOfArrival', [0; 0.2; 0.4; 0.6; 0.8]);
            %
            %   % Start the simulation loop
            %   while advance(scene)
            %      % get position of all other platforms in the first platform's body frame.
            %      poses = targetPoses(plat1);
            %      p = poses(1).Position;
            %
            %      % display the second platform's location relative to the first
            %      fprintf('plat2''s location as seen from plat1 is [%f %f]\n', p(1), p(2));
            %   end
            %
            %   See also:  platformPoses, trackingScenario/platform.
            targetsOut = targetPoses@radarfusion.internal.scenario.Platform(obj, varargin{:});
        end
        
        function p = pose(obj, varargin)
            %POSE  Return pose information for the platform
            %   p = POSE(plat) returns the platform's pose estimated
            %   by the pose estimator set in the platform's 'PoseEstimator'
            %   property.
            % 
            %   p = POSE(plat, type) returns the platform's pose based on
            %   the type of pose requested, where type can be either
            %   'estimated' or 'true'. When 'estimated' is selected, the
            %   pose is estimated using the pose estimator set in the
            %   platform's 'PoseEstimator' property. When 'true' is
            %   selected, the true pose of the platform is returned. By
            %   default, 'estimated' is selected.
            % 
            %   p = POSE(..., 'CoordinateSystem', coord) allows you to
            %   specify the coordinate system used to report the Position
            %   field. This optional syntax is only valid when scene.IsEarthCentered
            %   is true. Accepted values for coord are:
            %       'cartesian'  (default) - ECEF cartesian coordinates
            %       'geodetic'             - Geodetic coordinates
            % 
            %   See also: insSensor, trackingScenario/platformPoses
            p = pose@radarfusion.internal.scenario.Platform(obj, varargin{:});
        end
        
        function [signals, configs] = emit(obj, time)
            %EMIT  return the emissions generated by emitters mounted on the platform
            %  [emiss, configs] = EMIT(plat, time) reports a cell array of
            %  emissions and a struct array of the emitter configurations,
            %  configs, from the emitters attached to the platform, plat.
            %
            % See also: pose, detect, radarEmitter, sonarEmitter
            narginchk(2,2);
            [signals, configs] = emit@radarfusion.internal.scenario.Platform(obj, time);
        end
        
        function [detections,numDets,configs] = detect(obj, varargin)
            %DETECT  return the detections generated by sensors mounted on the platform
            %  detections = DETECT(plat, time) reports the detections from
            %  all sensors mounted on the platform, plat, that do not
            %  process emissions returned by emit(plat).
            %
            %  detections = DETECT(plat, signals, time) reports the
            %  detections from the sensors mounted on the platform, plat,
            %  that process emissions without information about emitter
            %  configurations (i.e. passive and bistatic sensors).  
            %
            %  detections = DETECT(plat, signals, configs, time) reports
            %  the detections from all sensors mounted on the platform,
            %  plat, including those that require information about emitter
            %  configurations (i.e. some monostatic sensor model
            %  configurations such as those that model cross-talk between
            %  sensors).
            %
            %  Detections are always returned as a cell array of
            %  objectDetection objects.
            %
            %  [...,numDets] = DETECT(...), additionally, returns the
            %  number of valid detections, numDets.
            %
            %  [...,configs] = DETECT(...), additionally, returns a struct
            %  array of the configurations of each sensor at the detection
            %  time.
            %
            % See also: pose, emit, fusionRadarSensor, sonarSensor,
            %           irSensor, objectDetection
            narginchk(2,4);
            [detections,numDets,configs] = detect@radarfusion.internal.scenario.Platform(obj, varargin{:});
        end
    end
    
    methods (Static,Hidden)
        function validateMesh(s)
            validateattributes(s,{'extendedObjectMesh'},{'scalar'},mfilename,'Mesh');
        end
    end
    
    methods (Access = {?radarfusion.internal.scenario.Scenario, ?radarfusion.internal.scenario.Platform})
        function newPlat = clone(obj,newScenario)
            %CLONE Create a copy of the platform
            %  newPlat = CLONE(obj,newScenario) returns a copy of the
            %  platform created 
            newPlat = fusion.scenario.Platform(newScenario,obj.PlatformID);
            cloneProperties(newPlat, obj);
        end
    end
    
    methods (Access = protected)
        function params = defaultConstruction(obj)
            params = defaultConstruction@radarfusion.internal.scenario.Platform(obj);
            params.DefaultMesh = defaultMesh(obj);
        end
        
        function out = defaultMesh(~)
            out = extendedObjectMesh('cuboid');
        end
        
        function out = defaultSignatures(~)
            out = {rcsSignature, irSignature, tsSignature};
        end
    end
    
    methods
        function tgtMeshesOut = targetMeshes(obj, reportSelf, fmt)
            % tgtMeshes = targetMeshes(plat) returns the relative poses of all
            % other platforms along with their mesh. The output of this
            % function can be used as an input for simulating lidar data
            % using monostaticLidarSensor.
            %
            % ... = targetMeshes(plat, reportSelf) allows you to report
            % information about platform, plat on the output. This allows
            % you to drive sensors which can detect the mounted platform as
            % well.
            %
            % ... = targetMeshes(plat, reportSelf, fmt) allows you to
            % specify the format of orientation as 'quaternion' or
            % 'rotmat'.
            %
            % See also: monostaticLidarSensor, targetPoses
            narginchk(1,3);
            
            if nargin < 2
                reportSelf = false;
            end
            
            if nargin<3
                fmt = 'quaternion';
            end
            fmt = validatestring(fmt,{'quaternion','rotmat'});
            
            egoPoseMesh = truePoseMesh(obj, fmt);
            
            targets = getTargets(obj);
            if reportSelf
                targets = [{obj} targets];
            end
            numTgts = numel(targets);
            nValid = 0;
            
            tgtMeshes = repmat(egoPoseMesh,numTgts,1);
            
            % report detections only if ego and target are valid
            if isValidPose(obj)
                for m = 1:numTgts
                    if isValidPose(targets{m})
                        nValid = nValid + 1;
                        tgtMeshes(nValid) = targetPoseMesh(obj, targets{m}, fmt);
                    end
                end
            end
            tgtMeshesOut = tgtMeshes(1:nValid);
        end
                
        function [ptCloud, configs, clusters] = lidarDetect(obj, time, includeSelf)
            % ptClouds = lidarDetect(plat, time) reports the point
            % clouds from all lidar sensors mounted on the platform, plat.
            %
            % ptClouds = lidarDetect(plat, time, includeSelf) allows you to
            % specify if the lidar sensors can detect the platform they are
            % mounted on. By default, includeSelf is false.
            %
            % [ptClouds, configs] = lidarDetect(...), additionally allows
            % you to output a struct array representing configurations of
            % the sensors
            %
            % [ptClouds, configs, clusters] = lidarDetect(...),
            % additionally allows you to output a cell array, clusters,
            % representing the ground truth information for each point.
            %
            % ptClouds is a cell array of point clouds, where each element
            % is a N-by-3 or P-by-Q-by-3 matrix defining the locations of
            % the point cloud. It is defined as a N-by-3 if
            % HasOrganizedOutput property of the sensor is set to false.
            %
            % configs is an array of struct, where each element defines the
            % configuration of the sensor. The fields of the struct are
            % defined <a href="matlab:help fusion.internal.interfaces.DataStructures/sensorConfigStruct">here</a>.
            %
            % clusters is a cell array of ground truth segmentation labels.
            % Each element of the cell array defines the cluster labels for
            % the corresponding point cloud.
            %
            % See also: monostaticLidarSensor, targetMeshes, pose, detect
            
            % Ensure obj is setup and locked
            setup(obj);
            
            if nargin == 2
                includeSelf = false;
            else
                validateattributes(includeSelf,{'numeric','logical'},{'binary','scalar'},'lidarDetect','includeSelf',3);
            end
            
            lidarSensors = selectLidarSensors(obj);

            numSensors = numel(lidarSensors);
            
            % If pose is invalid, set numSensors to zero. This will ensure
            % the output is empty cell arrays and structs.
            if ~isValidPose(obj)
                numSensors = 0;
            end
            
            ptCloud = cell(numSensors,1);
            clusters = cell(numSensors,1);
            configs = repmat(monostaticLidarSensor.sampleConfiguration,numSensors,1);
            
            if numSensors == 0
                return;
            end
           
            tgtMeshes = targetMeshes(obj, includeSelf);
            
            ins = pose(obj);
            
            for i = 1:numSensors
                thisSensor = lidarSensors{i};
                if thisSensor.HasINS
                    [ptCloud{i},configs(i),clusters{i}] = step(thisSensor,tgtMeshes, ins, time);
                else
                    [ptCloud{i},configs(i),clusters{i}] = step(thisSensor,tgtMeshes, time);
                end
            end
        end
    end
    
    methods (Access = {?radarfusion.internal.scenario.Scenario, ?radarfusion.internal.scenario.Platform, ?matlab.unittest.TestCase})
        function scaleMesh(obj)
            if ~isempty(obj.Mesh)
                isPoint = isPointPlatform(obj);
                dims = obj.Dimensions;
                if isPoint
                    % Preserve data type using subscripted assignment
                    dims.Length(1) = max(1e-6,dims.Length);
                    dims.Width(1) = max(1e-6,dims.Width);
                    dims.Height(1) = max(1e-6,dims.Height);
                end
                obj.Mesh = scaleToFit(obj.Mesh, dims);
            end
        end
        
        function lidars = selectLidarSensors(obj)
            sensors = obj.Sensors;
            isLidar = cellfun(@(x)isa(x,'monostaticLidarSensor'),sensors);
            lidars = sensors(isLidar);
        end
    end
    
    methods (Access = protected)
        function useNewDimensions(obj)
            scaleMesh(obj);
        end
    end
        
    methods(Hidden)
        function val = truePoseMesh(obj, fmt)
            if nargin<2
                argin = {};
            else
                argin = {fmt};
            end
            
            % A subset of targetPose and Mesh
            val = struct( ...
                'PlatformID', obj.PlatformID, ...
                'ClassID', obj.ClassID, ...
                'Position', position(obj), ...
                'Orientation', orientation(obj, argin{:}),...
                'Mesh',obj.Mesh...
                );
        end
        
        function val = targetPoseMesh(obj, tgt, fmt)
            if nargin < 3
                argin = {};
            else
                argin = {fmt};
            end
            val = truePoseMesh(tgt, argin{:});
            % Convert Position and Orientation w.r.t to Ego
            tgtPose = radarfusion.internal.scenario.targetsToEgo(truePose(tgt),truePose(obj, argin{:}));
            val.Position = tgtPose.Position;
            val.Orientation = tgtPose.Orientation;
        end
    end
    
    % Display
    methods (Access = protected)
       function groups = getPropertyGroups(~)
            propList = {'PlatformID','ClassID','Position','Orientation','Dimensions','Mesh','Trajectory', ...
                'PoseEstimator','Emitters','Sensors', ...
                'Signatures'};
            groups = matlab.mixin.util.PropertyGroup(propList);
       end
    end
    
    methods (Access = protected)
        function reload(obj, s)
            reload@radarfusion.internal.scenario.Platform(obj,s);
            if isprop(s,'Mesh') || isfield(s,'Mesh')
                obj.Mesh = s.Mesh;
            else
                obj.Mesh = defaultMesh(obj);
            end
        end
        
        function cloneProperties(obj, oldObj)
            cloneProperties@radarfusion.internal.scenario.Platform(obj, oldObj);
            obj.Mesh = oldObj.Mesh;
        end
    end
    
    methods (Static)
        function obj = loadobj(s)
            obj = fusion.scenario.Platform(trackingScenario,s.PlatformID);
            reload(obj, s);
        end
    end
end




