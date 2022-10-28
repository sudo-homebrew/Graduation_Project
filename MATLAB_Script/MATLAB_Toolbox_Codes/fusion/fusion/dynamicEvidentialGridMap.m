classdef (Sealed) dynamicEvidentialGridMap < handle
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
    
    %#codegen
    properties (SetAccess = protected)
        NumStateVariables
        MotionModel
    end
    
    properties (Dependent, SetAccess = protected)
        GridLength
        GridWidth
        GridResolution
        GridOriginInLocal
    end
    
    properties (Access = {?dynamicEvidentialGridMap,?matlab.unittest.TestCase})
        MultilayerMap
        LocalCoordinateData
    end
    
    properties (Constant, Access = protected)
        OccupiedEvidenceLayerName = 'OccupancyEvidence';
        FreeEvidenceLayerName     = 'FreeEvidence';
        StateLayerName            = 'State';
        StateCovarianceLayerName  = 'StateCovariance';
        IsDynamicLayerName        = 'IsDynamic';
        XYCellLayerName           = 'XYLocations';
    end
    
    % Thresholds for cell to be declared dynamic
    properties (Constant, Access = protected)
        MinOccupiedEvidence = 0.5;
        MinDistance = 6;
    end
    
    properties (Access = protected, Dependent)
        VelocityIndex
    end
    
    properties (Access = protected)
        % Cell width to go into dynamic cell structure
        CellWidth;
        
        % A variable of the right data type to easily create values using
        % 'like'. Otherwise, we need to always do if else for gpuArray
        DataTypeVariable;
        
        % A non-tunable property to control memory allocation for dynamic
        % cells
        MaxNumGridCells
    end
    
    % Some objects need this to initialize memory
    properties (Hidden)
        GridSize
    end
    
    %% Getters/Setters
    methods
        function val = get.VelocityIndex(obj)
            switch obj.NumStateVariables
                case 4
                    val = [2 4];
                case 5
                    val = [2 4];
                case 6
                    val = [2 5];
                otherwise
                    coder.internal.assert(false,'fusion:dynamicEvidentialGridMap:UnsupportedModel',obj.NumStateVariables);
            end
        end
        
        function val = get.GridSize(obj)
            val = obj.MultilayerMap.GridSize;
        end
        
        function val = get.GridLength(obj)
            val = diff(obj.MultilayerMap.XLocalLimits);
        end
        
        function val = get.GridWidth(obj)
            val = diff(obj.MultilayerMap.YLocalLimits);
        end
        
        function val = get.GridResolution(obj)
            val = obj.MultilayerMap.Resolution;
        end
        
        function val = get.GridOriginInLocal(obj)
            val = obj.MultilayerMap.GridOriginInLocal;
        end
    end
    
    methods
        function obj = dynamicEvidentialGridMap(L, W, res, varargin)
            % Allow constructing with an instance of itself.
            if isa(L,'dynamicEvidentialGridMap')
                copyPrivateProperties(obj, L);
                return;
            end
            
            % Parse inputs
            [motionModel,dataType,numStates,nvPairs] = dynamicEvidentialGridMap.parseInputs(varargin{:});
            
            % Create occupied evidence layer
            occEvidenceLayer = dynamicEvidentialGridMap.createOccupancyEvidenceLayer...
                (L, W, res, dataType, nvPairs{:});
            
            % Create free evidence layer
            freeEvidenceLayer = dynamicEvidentialGridMap.createFreeEvidenceLayer...
                (L, W, res, dataType, nvPairs{:});
            
            % Create state layer
            stateLayer = dynamicEvidentialGridMap.createStateLayer...
                (L, W, res, dataType, numStates, nvPairs{:});
            
            % Create state covariance layer
            stateCovarianceLayer = dynamicEvidentialGridMap.createStateCovarianceLayer...
                (L, W, res, dataType, numStates, nvPairs{:});
            
            % Create is dynamic layer
            isDynamicLayer = dynamicEvidentialGridMap.createIsDynamicLayer...
                (L, W, res, nvPairs{:});
            
            % Create the multi-layer map
            layers = {occEvidenceLayer;...
                freeEvidenceLayer;
                stateLayer;
                stateCovarianceLayer;
                isDynamicLayer};
            
            obj.MultilayerMap =  multiLayerMap(layers);
            
            % Create x, y location layer for storing local grid locations
            % of the cell centers. This is required to efficiently get the
            % x, y position of a cell in grid coordinates at the cost of
            % memory.
            obj.LocalCoordinateData = dynamicEvidentialGridMap.createXYLayer...
                (L, W, res, dataType, nvPairs{:});
            
            % Set MaxNumGridCells
            obj.MaxNumGridCells = ceil(L*W*res^2);
            
            % Set number of variables and motion model
            obj.NumStateVariables = numStates;
            obj.MotionModel = motionModel;
            
            % A variable which defines the data type
            obj.DataTypeVariable = obj.LocalCoordinateData.DefaultValue;
            
            % Store cell width as the same data type as locations.
            obj.CellWidth = cast(1/obj.GridResolution, 'like', obj.DataTypeVariable);
            
            % Set local coordinate data
            [i, j] = ind2sub(obj.GridSize, (1:prod(obj.GridSize))');
            p = cast(grid2local(obj.LocalCoordinateData, [i j]),'like',obj.DataTypeVariable);
            xCell = reshape(p(:,1),obj.GridSize);
            yCell = reshape(p(:,2),obj.GridSize);
            setMapData(obj.LocalCoordinateData,cat(3,xCell,yCell));
        end
        
        %% Public get methods
        function P_occ = getOccupancy(obj, varargin)
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
            
            validateInputs(obj, varargin{:});
            [m_occ, m_free] = getEvidences(obj, varargin{:});
            P_occ = m_occ + 0.5*(1 - m_occ - m_free);
        end
        
        function [m_occ, m_free] = getEvidences(obj, varargin)
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
            
            validateInputs(obj, varargin{:});
            m_occ = getOccupiedEvidence(obj,varargin{:});
            if nargout > 1
                m_free = getFreeEvidence(obj,varargin{:});
            end
        end
        
        function [v, Pv] = getVelocity(obj, varargin)
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
            %
            validateInputs(obj, varargin{:});
            v = getVelocityEstimate(obj,varargin{:});
            if nargout > 1
                Pv = getVelocityCovarianceEstimate(obj,varargin{:});
            end
        end
        
        function [x, P] = getState(obj ,varargin)
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
            %
            validateInputs(obj, varargin{:});
            x = getStateEstimate(obj, varargin{:});
            if nargout > 1
                P = getStateCovarianceEstimate(obj,varargin{:});
            end
        end
        
        function tf = isDynamic(obj, varargin)
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
            %
            validateInputs(obj, varargin{:});
            tf = getMapData(obj.MultilayerMap,obj.IsDynamicLayerName,varargin{:});
        end
        
        function show(map, varargin)
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
            
            % Parse flag inputs
            opArgs = {};
            poptions = struct('CaseSensitivity', true,...
                'IgnoreNulls',true,...
                'SupportOverrides', false);
            
            NVPairNames = {'FastUpdate', 'InvertColors', 'PlotVelocity', 'Parent'};
            pstruct = coder.internal.parseInputs(opArgs,NVPairNames,...
                poptions,varargin{:});
            fastUpdate = coder.internal.getParameterValue(pstruct.FastUpdate,true,varargin{:});
            invertColors = coder.internal.getParameterValue(pstruct.InvertColors,false,varargin{:});
            plotVelocity = coder.internal.getParameterValue(pstruct.PlotVelocity,false,varargin{:});
            
            % Parent supported in MATLAB execution only
            if coder.target('MATLAB')
                parentArgs = {coder.internal.getParameterValue(pstruct.Parent,gca,varargin{:})};
            else
                parentArgs = {};
            end
            
            % Gather map data based on flags
            [v, Pv] = getVelocity(map);
            vx = v(:,:,1);
            vy = v(:,:,2);
            Pxx = Pv(:,:,1,1);
            Pyy = Pv(:,:,2,2);
            Pxy = Pv(:,:,1,2);
            detS = Pxx.*Pyy - Pxy.^2;
            detS(detS < eps) = eps;
            d = (Pyy.*vx.^2 + Pxx.*vy.^2 - 2*Pxy.*vx.*vy)./detS;
            
            isDynamicCell = isDynamic(map);
            
            Pocc = getOccupancy(map);
            
            h = zeros(map.GridSize,'like',map.DataTypeVariable);
            s = zeros(map.GridSize,'like',map.DataTypeVariable);
            
            orient = mod(atan2d(vy(isDynamicCell),vx(isDynamicCell)),360);
            h(isDynamicCell) = orient./360;
            s(isDynamicCell) = min(1,d(isDynamicCell)/4);
            
            % Value must indicate probability of being "dynamic"
            v = Pocc;
            if ~invertColors
                v(~isDynamicCell) = 1 - v(~isDynamicCell);
            end
            
            hsvImage = cat(3,h,s,v);
            rgbImage = hsv2rgb(hsvImage);
            
            % If velocity needs to be plotter
            velData = struct;
            if plotVelocity
                xyCell = getMapData(map.LocalCoordinateData);
                xCell = xyCell(:,:,1);
                yCell = xyCell(:,:,2);
                velData.XData = gather(xCell(isDynamicCell));
                velData.YData = gather(yCell(isDynamicCell));
                velData.UData = gather(vx(isDynamicCell));
                velData.VData = gather(vy(isDynamicCell));
            end
            
            coder.extrinsic('fusion.internal.plotDynamicMapData');
            fusion.internal.plotDynamicMapData(map.GridLength,...
                map.GridWidth,...
                map.MultilayerMap.XLocalLimits,...
                map.MultilayerMap.YLocalLimits,...
                map.MultilayerMap.Resolution,...
                map.MultilayerMap.GridSize,...
                rgbImage,...
                velData,fastUpdate,plotVelocity,parentArgs{:});
        end
    end
    
    methods (Access = protected)
        function validateInputs(~, varargin)
            % Validate the number of inputs to get methods
            narginchk(1,3);
            if nargin > 1
                narginchk(3,3);
            end
        end
    end
    
    % Some additional protected get methods
    methods (Access = {?dynamicEvidentialGridMap,...
            ?fusion.internal.DynamicMapRFSFilter})
        function val = getStateEstimate(obj, varargin)
            val = getMapData(obj.MultilayerMap,obj.StateLayerName,varargin{:});
        end
        
        function val = getStateCovarianceEstimate(obj,varargin)
            val = getMapData(obj.MultilayerMap,obj.StateCovarianceLayerName,varargin{:});
        end
        
        function val = getVelocityEstimate(obj,varargin)
            x = getStateEstimate(obj,varargin{:});
            val = x(:,:,obj.VelocityIndex);
        end
        
        function val = getVelocityCovarianceEstimate(obj,varargin)
            P = getStateCovarianceEstimate(obj, varargin{:});
            val = P(:,:,obj.VelocityIndex,obj.VelocityIndex);
        end
        
        function val = getOccupiedEvidence(obj, varargin)
            val = getMapData(obj.MultilayerMap,obj.OccupiedEvidenceLayerName,varargin{:});
        end
        
        function val = getFreeEvidence(obj, varargin)
            val = getMapData(obj.MultilayerMap,obj.FreeEvidenceLayerName,varargin{:});
        end
    end
    
    %% Filter Estimation Assistance methods
    % Some methods for filter to interact with the map
    methods (Access = {?dynamicEvidentialGridMap,...
            ?fusion.internal.DynamicMapRFSFilter,...
            ?matlab.unittest.TestCase,...
            ?fusion.internal.DynamicEvidenceMap})
        % Set methods (only allow full update).
        function setOccupiedEvidence(obj, val)
            setMapData(obj.MultilayerMap,obj.OccupiedEvidenceLayerName,val);
        end
        
        function setFreeEvidence(obj, val)
            setMapData(obj.MultilayerMap,obj.FreeEvidenceLayerName,val);
        end
        
        function setState(obj,val)
            setMapData(obj.MultilayerMap,obj.StateLayerName,val);
        end
        
        function setStateCovariance(obj, val)
            setMapData(obj.MultilayerMap,obj.StateCovarianceLayerName,val);
        end
        
        function setIsDynamic(obj, val)
            setMapData(obj.MultilayerMap,obj.IsDynamicLayerName,cast(val,'like',obj.DataTypeVariable));
        end
        
        function setMapStateAndCovariance(obj, state, stateCov)
            setState(obj, state);
            setStateCovariance(obj, stateCov);
            tf = isDynamicCell(obj);
            setIsDynamic(obj, tf);
        end
        
        function tf = isDynamicCell(map)
            % tf = isDynamicCell returns information about the entire map
            % grid cells as a logical matrix. A true value indicates that a
            % cell is dynamic (and can be considered for object
            % extraction).
            
            % Get velocity of the entire grid
            [v, Pv] = getVelocity(map);
            
            % Compute mahalanobis distance with zero velocity
            ex = v(:,:,1);
            ey = v(:,:,2);
            Pxx = Pv(:,:,1,1);
            Pyy = Pv(:,:,2,2);
            Pxy = Pv(:,:,1,2);
            detS = Pxx.*Pyy - Pxy.^2;
            detS(detS < eps) = eps;
            d = (Pyy.*ex.^2 + Pxx.*ey.^2 - 2*Pxy.*ex.*ey)./detS;
            
            m_occ = getOccupiedEvidence(map);
            
            tf = d > map.MinDistance & m_occ > map.MinOccupiedEvidence;
        end
        
        function nullify(obj)
            ZERO = cast(0,'like',obj.DataTypeVariable);
            setOccupiedEvidence(obj,ZERO);
            setFreeEvidence(obj,ZERO);
            setState(obj,ZERO);
            setStateCovariance(obj,ZERO);
            setIsDynamic(obj,false);
        end
        
        function obj2 = clone(obj)
            obj2 = dynamicEvidentialGridMap(obj);
        end
        
        function copyPrivateProperties(obj, obj2)
            obj.MultilayerMap = copy(obj2.MultilayerMap);
            obj.LocalCoordinateData = copy(obj2.LocalCoordinateData);
            obj.CellWidth = obj2.CellWidth;
            obj.NumStateVariables = obj2.NumStateVariables;
            obj.MotionModel = obj2.MotionModel;
            obj.DataTypeVariable = obj2.DataTypeVariable;
            obj.MaxNumGridCells = obj2.MaxNumGridCells;
        end
    end
    
    methods (Hidden)
        function tf = isequal(obj,obj2)
            tf = builtin('isequal',obj,obj2);
        end
        
        function tf = isequaln(obj,obj2)
            tf = builtin('isequaln',obj,obj2);
        end
    end
    
    %% Object Extraction Assistance
    % Some methods for an object extractor to interact with the map
    methods (Access = {?dynamicEvidentialGridMap,...
            ?fusion.internal.GridObjectExtractor,...
            ?fusion.internal.GridMeasurementBuiltins,...
            ?matlab.unittest.TestCase})
        
        function ci = getCells(obj, cellIndices)
            if islogical(cellIndices)
                [i, j] = find(cellIndices);
            else
                [i, j] = ind2sub(obj.MultilayerMap.GridSize,cellIndices);
            end
            gridIndices = [i(:) j(:)];
            numCells = numel(i);
            if ~isempty(gridIndices)
                [m_occ, m_free] = getEvidences(obj,gridIndices,'grid');
                [x, P] = getState(obj,gridIndices,'grid');
                ciState = reshape(shiftdim(x,1),obj.NumStateVariables,numCells);
                ciStateCov = reshape(shiftdim(P,1),obj.NumStateVariables,obj.NumStateVariables,numCells);
            else
                m_occ = zeros(0,1,'like',obj.DataTypeVariable);
                m_free = zeros(0,1,'like',obj.DataTypeVariable);
                ciState = zeros(obj.NumStateVariables,0,'like',obj.DataTypeVariable);
                ciStateCov = zeros(obj.NumStateVariables,obj.NumStateVariables,0,'like',obj.DataTypeVariable);
            end
            ci = struct('CellWidth',obj.CellWidth,...
                'CellIndices',cellIndices,...
                'States',ciState,...
                'StateCovariances',ciStateCov,...
                'OccupancyEvidences',m_occ,...
                'FreeEvidences',m_free);
        end
        
        function ci = getSampleCell(obj)
            cIdx = [1;2];
            coder.varsize('cIdx',[obj.MaxNumGridCells 1],[1 0]);
            ci = getCells(obj,cIdx);
            ci.States(:) = 0;
            ci.StateCovariances(:,:,1) = eye(obj.NumStateVariables);
            ci.StateCovariances(:,:,2) = eye(obj.NumStateVariables);
            ci.OccupancyEvidences(:) = 1;
            ci.FreeEvidences(:) = 0;
        end
        
        function likelihood = gridTrackLikelihood(map, track)
            isDynamicCell = isDynamic(map);
            dynamicCells = getCells(map,isDynamicCell);
            likelihood = zeros(map.GridSize,'like',dynamicCells.CellWidth);
            
            switch map.NumStateVariables
                case 4
                    likelihood(isDynamicCell) = fusion.internal.GridMeasurementBuiltins.CVLikelihood(dynamicCells,track);
                case 5
                    likelihood(isDynamicCell) = fusion.internal.GridMeasurementBuiltins.CTLikelihood(dynamicCells,track);
                case 6
                    likelihood(isDynamicCell) = fusion.internal.GridMeasurementBuiltins.CALikelihood(dynamicCells,track);
            end
        end
    end
    
    methods (Access = private, Static)
        function [motionModel, dataType, numStates, otherArgs] = parseInputs(varargin)
            % Parse num state variables
            numVarsIdx = fusion.internal.findProp('NumStateVariables',varargin{:});
            if numVarsIdx < numel(varargin)
                numStates = varargin{numVarsIdx + 1};
                interArgs = {varargin{1:numVarsIdx-1},varargin{numVarsIdx+2:end}};
            else
                numStates = 4;
                interArgs = varargin;
            end
            
            % Parse data type
            dataTypeIdx = fusion.internal.findProp('DataType',interArgs{:});
            if dataTypeIdx < numel(interArgs)
                dataType = interArgs{dataTypeIdx + 1};
                otherArgs = {interArgs{1:dataTypeIdx-1},interArgs{dataTypeIdx+2:end}};
            else
                dataType = 'double';
                otherArgs = interArgs;
            end
            
            % motion model
            motionModel = dynamicEvidentialGridMap.getMotionModel(numStates);
        end
        
        function occLayer = createOccupancyEvidenceLayer(L, W, res, dataType, varargin)
            defaultValue = cast(0,dataType);
            layerName = dynamicEvidentialGridMap.OccupiedEvidenceLayerName;
            occLayer = mapLayer(L, W, 1, 'Resolution', res, 'DefaultValue', defaultValue, 'LayerName', layerName, varargin{:});
        end
        
        function freeLayer = createFreeEvidenceLayer(L, W, res, dataType, varargin)
            defaultValue = cast(0,dataType);
            layerName = dynamicEvidentialGridMap.FreeEvidenceLayerName;
            freeLayer = mapLayer(L, W, 1, 'Resolution', res, 'DefaultValue', defaultValue, 'LayerName', layerName, varargin{:});
        end
        
        function stateLayer = createStateLayer(L, W, res, dataType, numStates, varargin)
            defaultValue = cast(0,dataType);
            layerName = dynamicEvidentialGridMap.StateLayerName;
            stateLayer = mapLayer(L, W, numStates, 'Resolution', res, 'DefaultValue', defaultValue, 'LayerName', layerName, varargin{:});
        end
        
        function stateCovLayer = createStateCovarianceLayer(L, W, res, dataType, numStates, varargin)
            defaultValue = cast(0,dataType);
            layerName = dynamicEvidentialGridMap.StateCovarianceLayerName;
            stateCovLayer = mapLayer(L, W, [numStates numStates], 'Resolution', res, 'DefaultValue', defaultValue, 'LayerName', layerName, varargin{:});
        end
        
        function isDynamicLayer = createIsDynamicLayer(L, W, res, varargin)
            defaultValue = false;
            layerName = dynamicEvidentialGridMap.IsDynamicLayerName;
            isDynamicLayer = mapLayer(L, W, 1, 'Resolution', res, 'DefaultValue', defaultValue, 'LayerName', layerName, varargin{:});
        end
        
        function xyLayer = createXYLayer(L, W, res, dataType, varargin)
            defaultValue = cast(0, dataType);
            layerName = dynamicEvidentialGridMap.XYCellLayerName;
            xyLayer = mapLayer(L, W, 2, 'Resolution', res, 'DefaultValue', defaultValue, 'LayerName', layerName, varargin{:});
        end
        
        function val = getMotionModel(numStateVars)
            switch numStateVars
                case 4
                    val = 'constant-velocity';
                case 5
                    val = 'constant-turnrate';
                case 6
                    val = 'constant-acceleration';
                otherwise
                    coder.internal.assert(false,'fusion:dynamicEvidentialGridMap:UnsupportedModel',numStateVars);
            end
        end
    end
    
    methods (Static, Hidden)
        function props = matlabCodegenNontunableProperties(~)
            % Let the coder know about non-tunable parameters so that it
            % can generate more efficient code.
            props = {'NumStateVariables','MaxNumGridCells'};
        end
    end
    
     methods (Static)
        function sampleCell = getSampleCellSimulink(numStateVariables,dataType)
            sampleCell = struct('CellWidth',zeros(1,dataType), ...
                'CellIndices',zeros(2,1,dataType),'States',zeros(numStateVariables,2,dataType),...
                'StateCovariances',zeros(numStateVariables,numStateVariables,2,dataType), ...
                'OccupancyEvidences',zeros(2,1,dataType),'FreeEvidences',zeros(2,1,dataType));
        end
    end
end