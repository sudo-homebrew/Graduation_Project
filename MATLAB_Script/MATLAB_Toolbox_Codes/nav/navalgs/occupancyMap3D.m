classdef occupancyMap3D < handle
%occupancyMap3D Creates 3-D occupancy map
%   occupancyMap3D creates an occupancy map for three-dimensional
%   environments.
%
%   The occupancy map uses an Octree representation to model
%   environments using probabilities to indicate occupancy. You can
%   model an arbitrary environment that dynamically expands as
%   needed. The Octree representation trims data appropriately to
%   remain efficient both in memory and on disk.
%
%   OMAP = occupancyMap3D returns a 3D occupancy map object
%   initialized with default property values. This object can be used
%   to insert observations into a 3-dimensional environment
%   representation.
%
%   OMAP = occupancyMap3D(RES) creates a occupancyMap3D object
%   with map resolution, RES, in cells per meter.
%   RES is a scalar value with units of cells/meter.
%
%   OMAP = occupancyMap3D(RES, "PropertyName", PropertyValue)
%   creates a occupancyMap3D object with specified resolution as RES
%   and the other property values specified as name-value pair.
%
%   occupancyMap3D properties:
%       FreeThreshold          - Threshold to consider cells as obstacle-free
%       OccupiedThreshold      - Threshold to consider cells as occupied
%       ProbabilitySaturation  - Saturation limits on probability values as [min, max]
%                                NOTE: Only allowed to set during construction
%       Resolution             - Grid resolution in cells per meter
%                                NOTE: Only allowed to set during construction
%
%   occupancyMap3D methods:
%       checkOccupancy     - Check locations for free, occupied, unknown values
%       copy               - Create copy of object
%       getOccupancy       - Get occupancy probability of locations
%       inflate            - Inflate the map
%       insertPointCloud   - Insert 3D points or pointCloud observation in map
%       rayIntersection    - Compute map intersection points of rays
%       setOccupancy       - Set occupancy probability of locations
%       show               - Show occupancy map
%       updateOccupancy    - Update occupancy probability at locations
%
%   Example:
%
%       % Create a occupancyMap3D object.
%       omap = occupancyMap3D(10);
%
%       % Define a set of 3D points as observation from an origin pose ([x y z qw qx qy qz])
%       origin = [0 0 0 1 0 0 0];
%       points = repmat([0:0.25:2]', 1, 3);
%       maxRange = 5;
%
%       % Incorporate an observation.
%       insertPointCloud(omap, origin, points, maxRange);
%
%       % Visualize a 3D map of the environment.
%       show(omap);
%
%       % Inflate a map.
%       robotRadius = 0.2;
%       safetyRadius = 0.3;
%       inflationRadius = robotRadius + safetyRadius;
%       inflate(omap, inflationRadius);
%
%       % Query for a world coordinate occupancy.
%       probVal = getOccupancy(omap, [1 1 1]);
%
%       % Visualize the inflated 3D map of the environment.
%       show(omap);
%
%   See also occupancyMap, binaryOccupancyMap.

%   Copyright 2017-2021 The MathWorks, Inc.
%
%   References:
%
%   [1] Hornung, Armin, Kai M. Wurm, Maren Bennewitz, Cyrill Stachniss,
%   and Wolfram Burgard. "OctoMap: An efficient probabilistic 3D
%   mapping framework based on octrees." Autonomous Robots 34, no. 3
%   (2013): 189-206.

%#codegen

    properties (GetAccess = public, SetAccess = private)
        %ProbabilitySaturation Saturation limits for probability values
        ProbabilitySaturation = [0.001 0.999]
    end

    properties (Dependent, SetAccess = private)
        %Resolution Resolution of map in cells/meter
        % Default: 1 cells/meter
        Resolution
    end

    properties (Access = private)
        %OccupiedThresholdInternal Internal occupied threshold to consider location occupied
        %   This is needed to ensure the loading of the variable and
        %   validation of new incoming values are segregated.
        OccupiedThresholdInternal = 0.65

        %FreeThresholdInternal Internal free threshold to consider location free
        %   This is needed to ensure the loading of the variable and
        %   validation of new incoming values are segregated.
        FreeThresholdInternal = 0.2
    end

    properties (Access = public, Dependent)
        %OccupiedThreshold Threshold to consider location occupied
        OccupiedThreshold

        %FreeThreshold Threshold to consider location free
        FreeThreshold
    end

    properties (Access = {?occupancyMap3D, ?nav.algs.internal.InternalAccess, ?nav.algs.internal.MapIO})
        %Octree Stores octree data structure
        Octree = nav.algs.internal.OctomapWrapper.empty
    end

    properties (Access = {?occupancyMap3D, ?nav.algs.internal.InternalAccess})
        %OctreeDepth Maximum depth the underlying octree supports
        OctreeDepth = 16

        %PopulatedWorldLimits Based on inserted observations, what is the
        %populated world limits
        % NOTE: To support codegen for currentMapDimensions (which under
        % the hood calls the octree.getMapDimensions), need to initialize
        % the structure in which the map dimensions are stored
        PopulatedWorldLimits = struct('X', [0 0], 'Y', [0 0], 'Z', [0 0])

        %WorldLimits Based on the resolution set, what is the maximum world
        %limit which can be represented by the underlying octree data
        %representation
        WorldLimits

        %ProbMaxSaturation Maximum probability saturation possible
        ProbMaxSaturation = [0.001 0.999]
    end

    properties(Dependent, Access = {?occupancyMap3D, ?nav.algs.internal.InternalAccess})
        %MinimumVoxelSize The minimum size in m, of each voxel
        MinimumVoxelSize
    end


    methods
        %% Constructor
        function map = occupancyMap3D(res, varargin)
        %occupancyMap3D Constructor for class

        % validate resolution
            if mod(nargin,2) == 1
                % Ensure this is a valid resolution
                validateattributes(res, {'numeric', 'logical'}, ...
                                   {'scalar', 'real', 'positive', 'nonnan', 'finite'}, ...
                                   'occupancyMap3D', 'Resolution');
                resolution = double(res);
            else
                resolution = 1;
            end

            % Parse the varargins
            names = {'OccupiedThreshold', 'FreeThreshold', 'ProbabilitySaturation'};
            defaults = {0.65, 0.2, [0.001 0.999]};
            parser = robotics.core.internal.NameValueParser(names, defaults);
            parse(parser, varargin{:});

            % Extract the parsed values
            parsedOccupiedThreshold = parameterValue(parser, names{1});
            parsedFreeThreshold = parameterValue(parser, names{2});
            parsedProbabilitySaturation = parameterValue(parser, names{3});

            % Validate the values
            map.validateThreshold(parsedOccupiedThreshold, 'occupancyMap3D', 'OccupiedThreshold');
            map.validateThreshold(parsedFreeThreshold, 'occupancyMap3D', 'FreeThreshold');
            map.validateProbSaturation(parsedProbabilitySaturation, 'occupancyMap3D', 'ProbabilitySaturation');

            % Store the parsed values
            map.OccupiedThreshold = parsedOccupiedThreshold;
            map.FreeThreshold = parsedFreeThreshold;
            map.ProbabilitySaturation = parsedProbabilitySaturation;

            % Populate the internal properties
            map.Octree = nav.algs.internal.occupancyMap3DBuiltins(1/resolution);
            map.Octree.setClampingThreshold(map.ProbabilitySaturation(1), map.ProbabilitySaturation(2));
            map.computeWorldLimits;
        end

        function res = get.Resolution(map)
            res = 1.0/map.Octree.Resolution;
        end

        function minVoxelSize = get.MinimumVoxelSize(map)
            minVoxelSize = map.Octree.Resolution;
        end

        function set.OccupiedThreshold(map, newOccupiedThreshold)
        %set.OccupiedThreshold Set method for OccupiedThreshold
            map.validateThreshold(newOccupiedThreshold, 'set.OccupiedThreshold', 'OccupiedThreshold');

            freeThreshold = map.FreeThreshold;
            check = freeThreshold > newOccupiedThreshold;

            if check
                if coder.target('MATLAB')
                    error(message('nav:navalgs:occmap3d:ThresholdOutsideBounds', 'OccupiedThreshold', ...
                                  '>=', num2str(freeThreshold,'%.3f'), 'FreeThreshold'));
                else

                    coder.internal.errorIf(check, 'nav:navalgs:occmap3d:ThresholdOutsideBounds', 'OccupiedThreshold', ...
                                           '>=', coder.internal.num2str(freeThreshold), 'FreeThreshold');
                end
            end

            map.OccupiedThresholdInternal = newOccupiedThreshold;
        end

        function set.FreeThreshold(map, newFreeThreshold)
        %set.FreeThreshold Set method for FreeThreshold
            map.validateThreshold(newFreeThreshold, 'set.FreeThreshold', 'FreeThreshold');

            occupiedThreshold = map.OccupiedThreshold;
            check = newFreeThreshold > occupiedThreshold;
            if check
                if coder.target('MATLAB')
                    error(message('nav:navalgs:occmap3d:ThresholdOutsideBounds', 'FreeThreshold', ...
                                  '<=', num2str(occupiedThreshold,'%.3f'), 'OccupiedThreshold'));
                else

                    coder.internal.errorIf(check, 'nav:navalgs:occmap3d:ThresholdOutsideBounds', 'FreeThreshold', ...
                                           '<=', coder.internal.num2str(occupiedThreshold), 'OccupiedThreshold');
                end
            end

            map.FreeThresholdInternal = newFreeThreshold;
        end

        function y = get.OccupiedThreshold(map)
            y = map.OccupiedThresholdInternal;
        end

        function y = get.FreeThreshold(map)
            y = map.FreeThresholdInternal;
        end

        %% GET methods
        function pOcc = getOccupancy(map, pos)
        %getOccupancy Get occupancy probability of locations
        %   POCC = getOccupancy(MAP, XYZ) returns an N-by-1 array of
        %   occupancy values for N-by-3 array, XYZ. Each row of the
        %   array XYZ corresponds to a point with [X Y Z] world coordinates.
            pos = map.validatePosition(pos, map.WorldLimits, map.WorldLimits, map.WorldLimits, 'getOccupancy', 'xyz');
            pOcc = map.Octree.getOccupancy(pos);
        end

        function tOcc = checkOccupancy(map, pos)
        %checkOccupancy Check locations for free, occupied, unknown values
        %   TOCC = checkOccupancy(MAP, XYZ) returns an N-by-1 array of
        %   occupancy status using OccupiedThreshold and FreeThreshold
        %   properties, for N-by-3 array, XYZ. Each row of the array
        %   XYZ corresponds to a point with [X Y Z] world coordinates.
        %   TOCC is the ternary occupancy status, where 0 refers to
        %   obstacle-free cells, 1 refers to occupied cells and -1
        %   refers to unknown cells.

            pos = map.validatePosition(pos, map.WorldLimits, map.WorldLimits, map.WorldLimits, 'checkOccupancy', 'xyz');

            val = map.Octree.getOccupancy(pos);
            tOcc = map.getOccupancyValueAsTribool(val);
        end

        function newMap = copy(map)
        %copy Create copy of object
            newMap = occupancyMap3D(map.Resolution, ...
                                    'ProbabilitySaturation', map.ProbabilitySaturation, ...
                                    'OccupiedThreshold', map.OccupiedThreshold, ...
                                    'FreeThreshold', map.FreeThreshold);
            newMap.Octree.deserialization(map.Octree.serialization());
        end

        %% SET Methods (Functionality)
        function setOccupancy(map, pos, newOccupancy)
        %setOccupancy Set occupancy probability of locations
        %   setOccupancy(MAP, XYZ, POCC) assigns the occupancy
        %   values, POCC to the XYZ coordinate. POCC can be a N-by-1
        %   vector or scalar. The scalar is applied to all points in
        %   XYZ with each row specified as [x y z] coordinates

        % Validate values
            pos = map.validatePosition(pos, map.WorldLimits, map.WorldLimits, map.WorldLimits, 'setOccupancy', 'xyz');
            newOccupancy = map.validateOccupancyValues(newOccupancy, size(pos, 1), 'setOccupancy', 'pOcc');

            for idx = 1:size(pos, 1)
                map.Octree.setNodeValue(pos(idx,:), newOccupancy(idx), false);
            end
        end

        function updateOccupancy(map, pos, newObservation)
        %updateOccupancy Update occupancy probability at locations
        %   updateOccupancy(MAP, XYZ, OBS) probabilistically integrates the
        %   observations, OBS to the XYZ coordinates. OBS can be an N-by-1
        %   or scalar. The observations are applied to the corresponding
        %   rows in XYZ at the [x y z] coordinates.  A scalar observation
        %   is applied to all rows. If logical observations are provided,
        %   default update values are 0.7 and 0.4 for true and false
        %   respectively. OBS observations can be specified as numeric type
        %   with a value between 0 and 1.

        % Validate values
            pos = map.validatePosition(pos, map.WorldLimits, map.WorldLimits, map.WorldLimits, 'updateOccupancy', 'xyz');
            newObservation = map.validateOccupancyValues(newObservation, size(pos, 1), 'updateOccupancy', 'obs');

            for idx = 1:size(pos, 1)
                if islogical(newObservation(idx))
                    map.Octree.updateNodeBoolean(pos(idx, :), newObservation(idx), false);
                else
                    map.Octree.updateNodeDouble(pos(idx, :), newObservation(idx), false);
                end
            end
        end

        %% Map Operations
        function inflate(map, inflationRadius)
        %INFLATE Inflate the map
        %   INFLATE(MAP, R) inflates each occupied location of the
        %   occupancy map by R meters.

            validateattributes(inflationRadius, {'double'}, {'real', 'finite', 'positive'}, 'inflate', 'r');
            map.Octree.inflate(inflationRadius, map.OccupiedThreshold);
        end

        function insertPointCloud(map, pose, points, maxRange)
        %insertPointCloud Insert 3D points or pointCloud observation in map
        %   insertPointCloud(MAP, SENSORPOSE, POINTS, MAXRANGE) inserts
        %   a 3-dimensional set of points onto the OccupancyMap.
        %   SENSORPOSE is a 7-element vector representing sensor pose
        %   [X Y Z QW QX QY QZ] in the world coordinates. The
        %   orientation of the sensor pose is expected as a quaternion,
        %   with sequence [QW, QX, QY, QZ]. POINTS is a Nx3 matrix of
        %   3-dimensional points [X Y Z] per row in sensor coordinates.
        %   MAXRANGE is the maximum range of the sensor.
        %
        %   insertPointCloud(MAP, SENSORPOSE, POINTCLOUDOBJ, MAXRANGE)
        %   inserts a scalar pointCloud object in OccupancyMap.
        %   POINTCLOUDOBJ represents scalar pointCloud object as
        %   observed from the sensor, and expressed in sensor coordinates.

        % Validate pose

        % To use validatePose code when available.
            validPose = robotics.internal.validation.validateVectorNumElements(pose, 7, 'insertPointCloud', 'sensorPose');

            validPose(1:3) = map.validatePosition(validPose(1:3), map.WorldLimits, map.WorldLimits, map.WorldLimits, 'insertPointCloud', 'sensorPose');

            % Validate maxrange
            validateattributes(maxRange, {'double'}, {'scalar', 'real', 'finite'}, 'insertPointCloud', 'maxRange');

            if isa(points, 'pointCloud')
                validateattributes(points, {'pointCloud'}, {'scalar'}, 'insertPointCloud', 'pointCloudObj');
                % Extract location of the pointCloud and reshape into M x 3
                % NOTE: pointCloud can be of Mx3 or MxNx3 size
                points = reshape(points.Location, [], 3);
            else
                validateattributes(points, {'double', 'single'}, {'2d', 'ncols', 3, 'nonempty', 'real'}, 'insertPointCloud', 'points');
            end
            map.Octree.insertPointCloud(validPose, points, maxRange, false, false);
        end
        
        function [pts, isOccupied] = rayIntersection(map, sensorPose, directions, maxRange, ignoreUnknownCells)
            %rayIntersection Compute map intersection points of rays
            %   [PTS, OCCUPIED] = rayIntersection(MAP, SENSORPOSE,
            %   DIRECTIONS) returns end points, PTS, in the world
            %   coordinate frame for rays emanating from SENSORPOSE along
            %   given DIRECTIONS and the end points occupancy status,
            %   OCCUPIED. PTS is an N-by-3 array of points that is closest
            %   to the origin on the ending cell surfaces. SENSORPOSE is a
            %   7-element vector representing sensor pose [X Y Z QW QX QY
            %   QZ] in the world coordinates. OCCUPIED is an N-by-1 vector,
            %   where each row corresponds to one ray's end point occupancy
            %   status. 0 indicates an obstacle-free cell, 1 is occupied,
            %   and -1 is unknown. DIRECTIONS can be either an N-by-3 array
            %   of directional vectors [DX DY DZ] in sensor coordinate
            %   frame or an N-by-2 array of directions expressed as [AZ
            %   EL]. AZ is azimuth angle measured from +x direction to +y
            %   direction in sensor coordinate frame. EL is elevation angle
            %   measured from xy-plane to +z direction in sensor
            %   coordinate frame.
            %
            %   [PTS, OCCUPIED] = rayIntersection(__, MAXRANGE,
            %   IGNOREUNKNOWN) additionally accepts optional arguments
            %   MAXRANGE and IGNOREUNKNOWN. MAXRANGE is a scalar
            %   representing the maximum range of the rays. If there is no
            %   collision up to the maximum range, then the last cell of
            %   the ray endpoint is returned in PTS. By default, MAXRANGE
            %   is -1, which extends the ray to the  map boundary.
            %   IGNOREUNKNOWN is a scalar logical value that indicates
            %   whether cells of unknown occupancy would be treated as free
            %   when computing intersecting points between ray and map
            %   cells. If it is false, the unknown cells are treated as
            %   occupied cells and rays can be intersected by the unknown
            %   cells. By default, IGNOREUNKNOWN is true indicating unknown
            %   cells are considered obstacle free.
            %
            %   Example:
            %
            %      map = occupancyMap3D();
            %      setOccupancy(map, eye(3)*10, [1;1;1]);
            %      % perform ray intersection at origin with direction
            %      % along x, y, z axes
            %      [pts, isOccupied] = rayIntersection(map, ...
            %          [0 0 0 1 0 0 0], eye(3))
            %
            %      % perform ray intersection at origin with direction
            %      % along x, y, z axes using azimuth and elevation angles
            %      [pts, isOccupied] = rayIntersection(map, ...
            %          [0 0 0 1 0 0 0], [0 0; pi/2 0; 0 pi/2])  
            %
            %      % perform ray intersection with limited range
            %      [pts, isOccupied] = rayIntersection(map, ...
            %          [0 0 0 1 0 0 0], [0 0; pi/2 0; 0 pi/2], 5)
            %
            %      % perform ray intersection while treating unknown cells
            %      % as occupied
            %      [pts, isOccupied] = rayIntersection(map, ...
            %          [0 0 0 1 0 0 0], [0 0; pi/2 0; 0 pi/2], -1, false)
            
            narginchk(3,5);
            if nargin < 5
                ignoreUnknownCells = true;
            else
                validateattributes(ignoreUnknownCells, {'logical'}, {'scalar'}, 'rayIntersection', 'ignoreUnknownCells');
            end
            
            if nargin < 4
                maxRange = -1;
            else
                validateattributes(maxRange, {'numeric'}, {'scalar'}, 'rayIntersection', 'maxRange');
                if (maxRange <= 0 && maxRange ~= -1)
                    coder.internal.error('nav:navalgs:occmap3d:RayIntersectionMaxRangeUnsupported');
                end
                if isinf(maxRange)
                    maxRange = -1;
                end
                maxRange = double(maxRange);
            end
            
            validPose = robotics.internal.validation.validateVectorNumElements(sensorPose, 7, 'rayIntersection', 'sensorPose');
            origin = map.validatePosition(validPose(1:3), map.WorldLimits, map.WorldLimits, map.WorldLimits, 'rayIntersection', 'sensorPose');
            validateattributes(directions, {'numeric'}, {'2d', 'nonempty'}, 'rayIntersection', 'directions');
            
            R = quat2rotm(double(validPose(4:7)));
            if size(directions,2) == 3
                if any(all(directions == 0, 2))
                    coder.internal.error('nav:navalgs:occmap3d:RayIntersectionDirectionAllZeros');
                end
                raycastDirections = (R*directions')';
            else
                if size(directions, 2) ~= 2
                    coder.internal.error('nav:navalgs:occmap3d:RayIntersectionDirectionWrongSize');
                end
                az = double(directions(:,1));
                el = double(directions(:,2));
                raycastDirections = (R*[cos(az) sin(az) tan(el)]')';
            end
            
            out = map.Octree.getRayIntersection(double(origin), raycastDirections, map.OccupiedThreshold, ignoreUnknownCells, maxRange);
            
            isOccupied = map.getOccupancyValueAsTribool(out(:,1));
            pts = min(max(out(:,2:4), map.WorldLimits(1)), map.WorldLimits(2));
        end

        %% Visualization
        function varargout = show(map, varargin)
        %show Show occupancy map
        %   This method plots the three-dimensional environment using 3-D
        %   voxels whose occupancy values are greater than the
        %   specified OccupiedThreshold property of the map object. The
        %   color of the three-dimensional plot is strictly height based
        %
        %   HAXES = show(MAP) returns the handle to
        %   the plot axes as HAXES by show method.
        %
        %   show(MAP,Name,Value) provides additional options specified
        %   by one or more Name,Value pair arguments. Name must appear
        %   inside single quotes (''). You can specify several name-value
        %   pair arguments in any order as Name1,Value1,...,NameN,ValueN:
        %
        %       'Parent'        - Handle of an axes that specifies
        %                         the parent of the image object
        %                         created by show.

            if ~coder.target('MATLAB')
                %  Code generation

                % Always throw error when calling show in
                % generated code
                coder.internal.errorIf(~coder.target('MATLAB'),...
                                       'nav:navalgs:occmap3d:GraphicsSupportCodegen','show');
            end

            % Defaults
            defaults.Parent = [];

            parser = inputParser();
            parser.addParameter('Parent', defaults.Parent, @robotics.internal.validation.validateAxesUIAxesHandle);

            parse(parser, varargin{:});

            % Ensure valid parent
            axHandle = parser.Results.Parent;
            if isempty(axHandle)
                axHandle = newplot;
            end

            % Extract data
            visData = map.Octree.extractVisualizationData(16);
            origin = visData(:,1:3);
            scale = visData(:,4);
            value = visData(:,5);

            % Extract populated world limits
            pWorldLimits = map.currentMapDimensions();
            midWorldLimit = mean(pWorldLimits, 2);
            minWorldLimit = min(pWorldLimits(:,1));
            maxWorldLimit = max(pWorldLimits(:,2));
            if maxWorldLimit <= minWorldLimit
                maxWorldLimit = minWorldLimit + 1;
            end
            deltaWorldLimit = (maxWorldLimit - minWorldLimit)/2;

            % 3D plotting
            numLeaves = size(visData,1);
            numFacesPerNode = 12;
            numVerticesPerNode = 8;
            faces = zeros(numFacesPerNode*numLeaves,3);
            vertices = zeros(numVerticesPerNode*numLeaves,3);
            colors = zeros(numVerticesPerNode*numLeaves,1);
            j = 1;
            for i = 1:numLeaves
                if value(i) < -inf|| value(i) > map.OccupiedThreshold
                    [fi, vertices(numVerticesPerNode*(j-1) + (1:numVerticesPerNode),:)] = map.voxelPatchData(origin(i,:), scale(i));
                    faces(numFacesPerNode*(j-1) + (1:numFacesPerNode),:) = fi + numVerticesPerNode*(j-1);
                    colors(numVerticesPerNode*(j-1) + (1:numVerticesPerNode)) = origin(i,3)*ones(numVerticesPerNode,1);
                    j = j+1;
                end
            end
            faces = faces(1:numFacesPerNode*(j-1),:);
            vertices = vertices(1:numVerticesPerNode*(j-1),:);
            colors = colors(1:numVerticesPerNode*(j-1));

            % In order to be able to add data to existing occupancy map
            % plot, ensure the patch object with the correct tag is
            % present.
            hPatch = findobj(axHandle, 'Tag', message('nav:navalgs:occmap3d:FigureTitle').getString);

            if isempty(hPatch)
                hPatch = patch(axHandle, 'Faces', faces, 'Vertices', vertices, ...
                               'FaceVertexCData', colors, 'FaceAlpha', 1, ...
                               'FaceColor', 'flat', 'EdgeColor', 'none', ...
                               'Tag', message('nav:navalgs:occmap3d:FigureTitle').getString); %#ok<NASGU>

                % Set the 3-D view
                axHandle.View = [-50 50];
                hLight = light(axHandle); %#ok<NASGU>

                % Set title and labels
                title(axHandle, ...
                      message('nav:navalgs:occmap3d:FigureTitle').getString);
                xlabel(axHandle, ...
                       message('nav:navalgs:occmap3d:FigureXLabel').getString);
                ylabel(axHandle, ...
                       message('nav:navalgs:occmap3d:FigureYLabel').getString);
                zlabel(axHandle, ...
                       message('nav:navalgs:occmap3d:FigureZLabel').getString);
            else
                hPatch.Faces = faces;
                hPatch.Vertices = vertices;
                hPatch.FaceVertexCData = colors;
            end
            % Set limits
            axHandle.XLim = [midWorldLimit(1)-deltaWorldLimit midWorldLimit(1)+deltaWorldLimit];
            axHandle.YLim = [midWorldLimit(2)-deltaWorldLimit midWorldLimit(2)+deltaWorldLimit];
            axHandle.ZLim = [midWorldLimit(3)-deltaWorldLimit midWorldLimit(3)+deltaWorldLimit];

            % Outputs
            if nargout == 1
                varargout{1} = axHandle;
            end
        end
    end

    methods (Access = {?occupancyMap3D, ?nav.algs.internal.InternalAccess, ?nav.algs.internal.MapIO})

        function read(map, filename)
        %read Read a Binary Tree (BT) or Octree (OT) format based on
        %extension of the file
            if ~exist(filename, 'file')
                error(message('nav:navalgs:occmap3d:FileNotFound', filename));
            end

            [~, ~, ext] = fileparts(filename);

            if strcmpi(ext, '.ot')
                map.Octree.read(filename);
            elseif strcmpi(ext, '.bt')
                map.Octree.readBinary(filename);
            else
                error(message('nav:navalgs:occmap3d:FileExtensionIncorrect', ext));
            end
        end

        function write(map, filename)
        %write Write to a Binary Tree (BT) or Octree (OT) format based
        %on file extension
            [~, ~, ext] = fileparts(filename);

            if strcmpi(ext, '.ot')
                map.Octree.write(filename);
            elseif strcmpi(ext, '.bt')
                map.Octree.writeBinary(filename);
            else
                error(message('nav:navalgs:occmap3d:FileExtensionIncorrect', ext));
            end
        end

        function memoryUsed = memoryUsage(map)
        %memoryUsage Returns the memoryUsed by the underlying
        %representation of Octree in MB
            memoryUsed = double(map.Octree.memoryUsage)/1e6;
        end

        function mapD = currentMapDimensions(map)
        %currentMapDimensions Extract the current dimensions of the map
        %based on inserted observations
            mapD = map.Octree.getMapDimensions();
            map.PopulatedWorldLimits.X = mapD(1,:);
            map.PopulatedWorldLimits.Y = mapD(2,:);
            map.PopulatedWorldLimits.Z = mapD(3,:);
        end

        function wlimit = computeWorldLimits(map)
        %computeWorldLimits Compute world limit
        %   This return the distance limits in x-y-z directions which
        %   can be represented by the Octree. Note, the world limits
        %   are identical in all three dimensions
            wlimit = (2^(map.OctreeDepth-1))*map.MinimumVoxelSize;
            map.WorldLimits = [-wlimit wlimit];
        end

        function validateProbSaturation(map, values, fcnName, argName)
        %validateProbSaturation Validates the probability saturation
            validateattributes(values, {'double'}, {'vector', 'numel', 2, 'real', 'increasing', '>=', 0, '<=', 1}, ...
                               fcnName, argName);

            validateattributes(values(1), {'numeric'}, {'>=',map.ProbMaxSaturation(1), ...
                                '<=',0.5}, ...
                               'ProbabilitySaturation', 'lower saturation');

            validateattributes(values(2), {'numeric'}, {'>=',0.5, ...
                                '<=',map.ProbMaxSaturation(2)}, ...
                               'ProbabilitySaturation', 'upper saturation');
        end
        
        function tOcc = getOccupancyValueAsTribool(map, val)
            %getOccupancyValue convert occupancy value to tri-bool, 1 for
            %occupied, 0 for free, -1 for unknown
            
            tOcc = -ones(size(val));
            tOcc(val >= map.OccupiedThreshold) = 1;
            tOcc(val <= map.FreeThreshold) = 0;
        end
    end

    methods (Static, Access = private)
        function [faces, vertices] = voxelPatchData(origin, scale)
        %voxelPatchData Get triangulated faces and vertices of a voxel
        %   Returns the triangulated faces and vertices of the voxel
        %   which is centered at the ORIGIN and the dimensions are
        %   scaled based on the incoming SCALE
            vertices = [ 1,  1,  1; ...
                         -1,  1,  1; ...
                         -1, -1,  1; ...
                         1, -1,  1; ...
                         1,  1, -1; ...
                         -1,  1, -1; ...
                         -1, -1, -1; ...
                         1, -1, -1];
            vertices = 0.5*scale*vertices + origin;
            %   Triangular faces
            faces = [1, 2, 3;
                     3, 4, 1; ...
                     1, 5, 6;
                     6, 2, 1; ...
                     2, 6, 7;
                     7, 3, 2; ...
                     3, 7, 8;
                     8, 4, 3; ...
                     4, 8, 5;
                     5, 1, 4; ...
                     5, 6, 7;
                     7, 8, 5];
        end

        function validateThreshold(value, fcnName, argName)
        %validateThreshold Validates the Occupied/Free threshold
            validateattributes(value, {'double'}, {'scalar', 'real', '>=', 0, '<=', 1}, ...
                               fcnName, argName);
        end

        function modifiedValues = validateOccupancyValues(values, len, fcnName, argname)
        %validateOccupancyValues Validate occupancy value vector
        %   Validates the incoming VALUES vector with expected length
        %   as LEN. The occupancy VALUES can be boolean or values
        %   between 0 and 1.

        % check that the values are numbers in [0,1]
            if islogical(values)
                validateattributes(values, {'logical'}, ...
                                   {'real','vector', 'nonempty'}, fcnName, argname);
            else
                validateattributes(values, {'numeric'}, ...
                                   {'real','vector', 'nonnan', '<=',1,'>=',0, 'nonempty'}, fcnName, argname);
            end

            if isscalar(values)
                % If inputs is a scalar, then return a vector of same
                % values
                if islogical(values)
                    % logical scalar --> logical vector
                    modifiedValues = logical(true(len, 1).*values);
                else
                    % double scalar --> double vector
                    modifiedValues = ones(len, 1).*values;
                end

            else
                isSizeMismatch =  length(values) ~= 1 && length(values) ~= len;
                coder.internal.errorIf(isSizeMismatch,...
                                       'nav:navalgs:occmap3d:InputSizeMismatch');
                modifiedValues = values;
            end
        end

        function pos = validatePosition(pos, xlimits, ylimits, zlimits, fcnName, argName)
        %validatePosition Validate the position to be within world
        %limits

        % Validate the input format and type
            validateattributes(pos, {'numeric'}, ...
                               {'real', 'nonnan', 'finite', 'nonempty', '2d', 'ncols', 3}, fcnName, argName);

            pos = double(pos);

            % Validate if the input is within the world limits
            maxpos = max(pos, [], 1);
            minpos = min(pos, [], 1);

            isOutside  = (minpos(1,1) < xlimits(1,1) || minpos(1,2) < ylimits(1,1) ...
                          || minpos(1,3) < zlimits(1,1) ...
                          || maxpos(1,1) > xlimits(1,2) || maxpos(1,2) > ylimits(1,2)) ...
                || maxpos(1,3) > zlimits(1,2);

            if isOutside
                if coder.target('MATLAB')
                    error(message('nav:navalgs:occmap3d:CoordinateOutside', ...
                                  num2str(xlimits(1,1),'%.2f'), num2str(xlimits(1,2),'%.2f'), ...
                                  num2str(ylimits(1,1),'%.2f'), num2str(ylimits(1,2),'%.2f'), ...
                                  num2str(zlimits(1,1),'%.2f'), num2str(zlimits(1,2),'%.2f')));
                else
                    coder.internal.errorIf(isOutside, 'nav:navalgs:occmap3d:CoordinateOutside', ...
                                           coder.internal.num2str(xlimits(1,1)), coder.internal.num2str(xlimits(1,2)), ...
                                           coder.internal.num2str(ylimits(1,1)), coder.internal.num2str(ylimits(1,2)), ...
                                           coder.internal.num2str(zlimits(1,1)), coder.internal.num2str(zlimits(1,2)));
                end
            end
        end
    end

    methods (Static, Hidden)
        function map = loadobj(incomingMap)
        % Create an object using the properties of the incoming map
            map = occupancyMap3D(incomingMap.Resolution, ...
                                 'ProbabilitySaturation', incomingMap.ProbabilitySaturation, ...
                                 'FreeThreshold', incomingMap.FreeThreshold, ...
                                 'OccupiedThreshold', incomingMap.OccupiedThreshold);
            % Read the internal Octree by deserializing the char array
            map.Octree.deserialization(incomingMap.Octree);
        end
    end

    methods (Hidden)
        function savedMap = saveobj(map)
        % Copy properties
            savedMap.Resolution = map.Resolution;
            savedMap.ProbabilitySaturation = map.ProbabilitySaturation;
            savedMap.FreeThreshold = map.FreeThreshold;
            savedMap.OccupiedThreshold = map.OccupiedThreshold;
            % Ensure the internal octree representation is stored as a
            % serialized char array
            savedMap.Octree = map.Octree.serialization();
        end
    end
end
