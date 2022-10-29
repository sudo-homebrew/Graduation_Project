classdef dynamicEvidentialGridMap< handle
% dynamicEvidentialGridMap Dynamic grid map output from trackerGridRFS
% An object of dynamicEvidentialGridMap class represents the dynamic
% map estimate from the grid-based tracker, trackerGridRFS. You can
% visualize the dynamic map and obtain the estimated values by using
% its methods. See below for the list of methods:
%
% dynamicEvidentialGridMap methods:
%   getEvidences - Get estimated occupied and free evidences
%   getOccupancy - Get estimated occupancy probabilities
%   getVelocity  - Get estimated velocity and associated uncertainty
%   getState     - Get full estimated state and associated uncertainty
%   show         - visualize the dynamic map
%
% dynamicEvidentialGridMap properties
%   MotionModel         - Motion model used to estimate the map (read-only)
%   NumStateVariables   - Number of state variables corresponding to the motion model (read-only)
%   GridLength          - Dimensions of the grid along X (read-only)
%   GridWidth           - Dimensions of the grid along Y (read-only)
%   GridResolution      - Resolution of the grid (cells/m) (read-only)
%   GridOriginInLocal   - Location of left corner of grid in Local or
%                         Ego coordinates (read-only)
%
% Example: Obtain estimated values at the grid level
% --------------------------------------------------
% 	% Create a tracking scenario
%   scene = trackingScenario('UpdateRate',5,'StopTime',5);
%
%   % For reproducible results
%   rng(2021);
%
%   % Add a platform with the sensor
%   plat = platform(scene);
%   lidar = monostaticLidarSensor(1,'DetectionCoordinates','Body');
%
%   % Add two targets within -50 and 50
%   for i = 1:2
%       target = platform(scene);
%       x = 50*(2*rand - 1);
%       y = 50*(2*rand - 1);
%       vx = 5*(2*rand - 1);
%       vy = 5*(2*rand - 1);
%       target.Trajectory.Position = [x y 0];
%       target.Trajectory.Velocity = [vx vy 0];
%       target.Trajectory.Orientation = quaternion([atan2d(vy,vx),0,0],'eulerd','ZYX','frame');
%       target.Mesh = extendedObjectMesh('sphere');
%       target.Dimensions = struct('Length',4,...
%           'Width',4,...
%           'Height',2,...
%           'OriginOffset',[0 0 0]);
%   end
%
%   % Configuration of the sensor for tracking
%   config = trackingSensorConfiguration(1,...
%       'SensorLimits',[-180 180;0 100],...
%       'SensorTransformParameters',struct,...
%       'IsValidTime',true);
%
%   % Create a tracker
%   tracker = trackerGridRFS('SensorConfigurations',config,...
%       'AssignmentThreshold',5,...
%       'MinNumCellsPerCluster',4,...
%       'ClusteringThreshold',3);
%
%   % Advance scenario and run the tracker on lidar data
%   while advance(scene)
%       % Current time
%       time = scene.SimulationTime;
%
%       % Generate point cloud
%       tgtMeshes = targetMeshes(plat);
%       [ptCloud, config] = lidar(tgtMeshes, time);
%
%       % Format the data for the tracker
%       sensorData = struct('Time',time,...
%           'SensorIndex',1,...
%           'Measurement',ptCloud',...
%           'MeasurementParameters',struct...
%           );
%
%       % Call tracker using sensorData to obtain the map in addition
%       % to tracks
%       [tracks, ~, ~, map] = tracker(sensorData, time);
%
%       % Obtain the estimated occupancy probability of each cell
%       P_occ = getOccupancy(map);
%
%       % Obtain the estimated evidences for each cell
%       [m_occ, m_free] = getEvidences(map);
%
%       % Obtain the estimated velocity for each cell
%       [v, Pv] = getVelocity(map);
%
%       % Obtain the estimated state for each cell
%       [x, P] = getState(map);
%   end
%
% See also: trackerGridRFS, trackingScenario, monostaticLidarSensor

     
    % Copyright 2020 The MathWorks, Inc.

    methods
        function out=dynamicEvidentialGridMap
            % Allow constructing with an instance of itself.
        end

        function out=getEvidences(~) %#ok<STOUT>
            % [m_occ, m_free] = getEvidences(MAP) provides the occupied
            % evidence, m_occ, and free evidence, m_free, grid cells as a
            % matrix.
            %
            % [m_occ, m_free] = getEvidences(MAP, XY, 'local') returns an
            % N-by-1 array of evidence values for N-by-2 array, XY. Each
            % row of the array XY corresponds to a point with [X Y] local
            % coordinates.
            %
            % [m_occ, m_free] = getOccupancy(MAP, IJ, 'grid') returns an
            % N-by-1 array of evidence values for N-by-2 array, IJ. Each
            % row of the array IJ refers to the cell index [i j].
        end

        function out=getOccupancy(~) %#ok<STOUT>
            % P_occ = getOccupancy(MAP) provides the occupancy
            % probabilities of all grid cells as a matrix.
            %
            % P_occ = getOccupancy(MAP, XY, 'local') returns an N-by-1
            % array of occupancy values for N-by-2 array, XY. Each row of
            % the array XY corresponds to a point with [X Y] local
            % coordinates.
            %
            % P_occ = getOccupancy(MAP, IJ, 'grid') returns an N-by-1
            % array of occupancy values for N-by-2 array, IJ. Each row of
            % the array IJ refers to the cell index [i j].
        end

        function out=getSampleCellSimulink(~) %#ok<STOUT>
        end

        function out=getState(~) %#ok<STOUT>
            % x = getState(MAP) provides the state estimate for each
            % grid cell. The number of elements in the third dimension of x
            % represent the number of state variables.
            %
            % x = getState(MAP, XY, 'local') returns the state
            % estimate at certain grid locations defined by N-by-2 array,
            % XY. Each row of XY array corresponds to a point with [X Y]
            % local coordinates.
            %
            % x = getState(MAP, IJ, 'grid') returns the state
            % estimate for certain grid cells defined by N-by-2 array, IJ.
            % Each row of the IJ array refers to the grid index [I J].
            %
            % [x, P] = getState(...) also returns the uncertainty in the
            % estimated state.
        end

        function out=getVelocity(~) %#ok<STOUT>
            % v = getVelocity(MAP) provides the velocity estimate for each
            % grid cell. The first page represents the velocity estimate in
            % x direction of tracking coordinate frame and the second page
            % represents the velocity esimate in y direction of tracking
            % coordinate frame.
            %
            % v = getVelocity(MAP, XY, 'local') returns the velocity
            % estimate at certain grid locations defined by N-by-2 array,
            % XY. Each row of XY array corresponds to a point with [X Y]
            % local coordinates.
            %
            % v = getVelocity(MAP, IJ, 'grid') returns the velocity
            % estimate for certain grid cells defined by N-by-2 array, IJ.
            % Each row of the IJ array refers to the grid index [I J].
            %
            % [v, P] = getVelocity(...) also returns the uncertainty in the
            % estimated velocity.
        end

        function out=isDynamic(~) %#ok<STOUT>
            % tf = isDynamic(MAP) provides a true/false value if the cell
            % is estimated to be a dynamic cell.
            %
            % tf = isDynamic(MAP, XY, 'local') returns an N-by-1
            % array of true/false values for N-by-2 array, XY. Each row of
            % the array XY corresponds to a point with [X Y] local
            % coordinates.
            %
            % tf = isDynamic(MAP, IJ, 'grid') returns an N-by-1
            % array of true/false values for N-by-2 array, IJ. Each row of
            % the array IJ refers to the cell index [i j].
        end

        function out=show(~) %#ok<STOUT>
            % show(map) plots the dynamic occupancy grid map
            % in local coordinates. The cells which are declared static are
            % visualized using a gray-scale image, where the grayness
            % represents the occupancy probability of the cell. To
            % visualize dynamic cells, the tracker uses HSV (hue,
            % saturation, value) values on an RGB colormap. The HSV values
            % for each grid cell are calculated using the following
            % convention:
            %
            % Hue is defined as orientation/360, where orientation is the
            % direction of the velocity vector in degrees. As hue increases
            % from 0 to 1, the color transitions from red to orange,
            % yellow, green, cyan, blue, magenta and finally back to red.
            %
            % Saturation is defined as the Mahalanobis distance (d) between
            % the velocity distribution of the grid cell and zero velocity.
            % A cell with d > 4 is drawn with full saturation (equal to 1)
            %
            % Value is defined as the occupancy probability of the cell.
            %
            % show(map,Name,value) allows specifying
            % additional name-value pairs defined below.
            %
            % 'PlotVelocity'  - A flag to control if velocity vectors must
            %                   be plotted in the visualization. If
            %                   specified as true, the velocity vector for
            %                   each dynamic cell is plotter at the center
            %                   of the grid cell. The length of the vector
            %                   represents the velocity magnitude in
            %                   tracking frame.
            %
            %                   Default: false
            %
            % 'Parent'        - Axes to plot the map, specified as an axes
            %                   handle.
            %
            %                   Default: gca
            %
            % 'FastUpdate'    - Boolean value used to speed up show method
            %                   for existing map plots. If you have
            %                   previously plotted your map on the axes,
            %                   specify 1 to perform a lightweight update
            %                   to the map in the figure.
            %
            %                   Default: true
            %
            % 'InvertColors'  - Boolean value to specify if the colors of
            %                   the static occupied cells should be
            %                   inverted. In the default settings, empty
            %                   spaces are white and occupied spaces are
            %                   black. When this flag is specified as true,
            %                   the colors are inverted.
            %
            %                   Default: false
            %
            % See also: hsv2rgb, rgb2hsv
        end

        function out=validateInputs(~) %#ok<STOUT>
            % Validate the number of inputs to get methods
        end

    end
    properties
        % Cell width to go into dynamic cell structure
        CellWidth;

        % A variable of the right data type to easily create values using
        % 'like'. Otherwise, we need to always do if else for gpuArray
        DataTypeVariable;

        FreeEvidenceLayerName;

        GridLength;

        GridOriginInLocal;

        GridResolution;

        GridWidth;

        IsDynamicLayerName;

        % A non-tunable property to control memory allocation for dynamic
        % cells
        MaxNumGridCells;

        MinDistance;

        MinOccupiedEvidence;

        MotionModel;

        NumStateVariables;

        OccupiedEvidenceLayerName;

        StateCovarianceLayerName;

        StateLayerName;

        VelocityIndex;

        XYCellLayerName;

    end
end
