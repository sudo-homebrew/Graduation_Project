classdef (Sealed) occupancyMap < matlabshared.autonomous.internal.MapLayer & ...
    matlabshared.autonomous.core.internal.CustomDisplay
%OCCUPANCYMAP Create an occupancy grid map
%   OCCUPANCYMAP creates a 2D occupancy grid map. Each cell has
%   a value representing the probability of occupancy of that cell.
%   Probability values close to 1 represent certainty that the workspace
%   represented by the cell is occupied by an obstacle. Values close to 0
%   represent certainty that the workspace represented by the cell is not
%   occupied and is obstacle-free.
%
%   The probability values in the occupancy grid map are stored with
%   a precision of at least 1e-3 to reduce memory usage and allow creation 
%   of occupancy map objects to represent large workspace. The minimum and
%   maximum probability that can be represented are 0.001 and 0.999
%   respectively.
%
%   MAP = occupancyMap creates a 2D occupancy grid map object
%   occupying a world space of width(W) = 10, height(H) = 10, and with
%   Resolution = 1.
%
%   MAP = occupancyMap(W, H) creates a 2D occupancy grid map
%   object representing a world space of width(W) and height(H) in
%   meters. The default grid resolution is 1 cell per meter.
%
%   MAP = occupancyMap(W, H, RES) creates an occupancyMap
%   object with resolution(RES) specified in cells per meter.
%
%   MAP = occupancyMap(M, N, RES, 'grid') creates an
%   occupancyMap object and specifies a grid size of M rows and N columns.
%   RES specifies the cells per meter resolution.
%
%   MAP = occupancyMap(P) creates an occupancy map object
%   from the values in the matrix, P. The size of the grid matches the
%   matrix with each cell value interpreted from that matrix location.
%   Matrix, P, may contain any numeric type with values between zero(0)
%   and one(1).
%
%   MAP = occupancyMap(P, RES) creates an occupancyMap
%   object from matrix, P, with RES specified in cells per meter.
%
%   MAP = occupancyMap(SOURCEMAP) creates an occupancyMap using
%   values from another occupancyMap object.
%
%   MAP = occupancyMap(SOURCEMAP, RES) creates an occupancyMap using values 
%   from another occupancyMap object, but resamples the matrix to have 
%   the specified resolution = RES.
%
%   MAP = occupancyMap(___, Name, Value, ...)  specifies additional attributes
%   of the occupancyMap object with each specified property name set to the 
%   specified value. Name must appear inside single quotes (''). You can 
%   specify several name-value pair arguments in any order as 
%   Name1,Value1,...,NameN,ValueN. Properties not specified retain their 
%   default values.
%
%   occupancyMap properties:
%       FreeThreshold           - Threshold to consider cells as obstacle-free
%       OccupiedThreshold       - Threshold to consider cells as occupied
%       ProbabilitySaturation   - Saturation limits on probability values as [min, max]
%       GridSize                - Size of the grid in [rows, cols] (number of cells)
%       LayerName               - The name of this occupancyMap instance
%       Resolution              - Grid resolution in cells per meter
%       XLocalLimits            - Min and max values of X in local frame
%       YLocalLimits            - Min and max values of Y in local frame
%       XWorldLimits            - Min and max values of X in world frame
%       YWorldLimits            - Min and max values of Y in world frame
%       GridLocationInWorld     - Location of the grid in world coordinates
%       LocalOriginInWorld      - Location of the local frame in world coordinates
%       GridOriginInLocal       - Location of the grid in local coordinates
%       DefaultValue            - Default value for uninitialized map cells
%
%   occupancyMap methods:
%       checkOccupancy  - Check ternary occupancy status for one or more positions
%       copy            - Create a copy of the object
%       getOccupancy    - Get occupancy value for one or more positions
%       grid2world      - Convert grid indices to world coordinates
%       grid2local      - Convert grid indices to local coordinates
%       inflate         - Inflate occupied positions by a given amount
%       insertRay       - Insert rays from laser scan observation
%       local2grid      - Convert local coordinates to grid indices
%       local2world     - Convert local coordinates to world coordinates
%       move            - Move map in world frame
%       occupancyMatrix - Export occupancyMap as a matrix
%       raycast         - Compute cell indices along a ray
%       rayIntersection - Compute map intersection points of rays
%       setOccupancy    - Set occupancy of a location
%       show            - Display the binary occupancy map in a figure
%       syncWith        - Sync map with overlapping map
%       updateOccupancy - Integrate probability observation at a location
%       world2grid      - Convert world coordinates to grid indices
%       world2local     - Convert world coordinates to local coordinates
%
%
%   Example:
%
%       % Create a 2x2 empty map.
%       map = occupancyMap(2,2);
%
%       % Create a 10x10 empty map with resolution 20.
%       map = occupancyMap(10, 10, 20);
%
%       % Insert a laser scan in the occupancy map.
%       ranges = 5*ones(100, 1);
%       angles = linspace(-pi/2, pi/2, 100);
%       insertRay(map, [5,5,0], ranges, angles, 20);
%
%       % Show occupancy grid map in Graphics figure.
%       show(map);
%
%       % Create a map from a matrix with resolution 20.
%       p = eye(100)*0.5;
%       map = occupancyMap(p, 20);
%
%       % Check occupancy of the world location [0.3 0.2].
%       value = getOccupancy(map, [0.3 0.2]);
%
%       % Set world position [1.5 2.1] as occupied
%       setOccupancy(map, [1.5 2.1], 0.8);
%
%       % Get the grid cell indices for world position [2.5 2.1].
%       ij = world2grid(map, [2.5 2.1]);
%
%       % Set the grid cell indices to unoccupied.
%       setOccupancy(map, [1 1], 0.2, 'grid');
%
%       % Integrate occupancy observation at a location.
%       updateOccupancy(map, [1 1], 0.7);
%
%       % Display occupancy map in a figure.
%       show(map);
%
%   See also monteCarloLocalization, binaryOccupancyMap.

%   Copyright 2016-2021 The MathWorks, Inc.

%#codegen

%%
    properties (Dependent)
        %OccupiedThreshold Threshold to consider cells as occupied
        %   A scalar representing the probability threshold above which
        %   cells are considered to be occupied. The OccupiedThreshold will be
        %   saturated based on ProbabilitySaturation property.
        %
        %   Default: 0.65
        OccupiedThreshold

        %FreeThreshold Threshold to consider cells as obstacle-free
        %   A scalar representing the probability threshold below which
        %   cells are considered to be obstacle-free. The FreeThreshold will be
        %   saturated based on ProbabilitySaturation property.
        %
        %   Default: 0.20
        FreeThreshold

        %ProbabilitySaturation Saturation limits on probability values as [min, max]
        %   A vector [MIN MAX] representing the probability saturation
        %   values. The probability values below MIN value will be saturated to
        %   MIN and above MAX values will be saturated to MAX. The MIN
        %   value cannot be below 0.001 and MAX value cannot be above 0.999.
        %
        %   Default: [0.001 0.999]
        ProbabilitySaturation
    end

    properties (Hidden, Constant)
        %DefaultType Default datatype used for log-odds
        DefaultType = 'int16';
    end

    properties (Access = {?occupancyMap, ?nav.algs.internal.InternalAccess}, Constant)
        %ProbMaxSaturation Maximum probability saturation possible
        ProbMaxSaturation = [0.001 0.999];

        %LookupTable Lookup table to convert between log-odds and probability
        LookupTable = createLookupTable()';

        %LinSpaceInt16 The line space for all integer log-odds values
        LinSpaceInt16 = getIntLineSpace();
        
        %GriddedInterpolant object to convert probability to logodds
        ProbInterpolant = getGriddedInterpolant();
        
        %ShowTitle Title for the show method
        ShowTitle = message('nav:navalgs:occgrid:FigureTitle').getString;
    end

    properties (Access = private)
        %FreeThresholdIntLogodds Integer log-odds values for FreeThreshold
        %   The log-odds value correspond to the default threshold of 0.2
        FreeThresholdIntLogodds

        %OccupiedThresholdIntLogodds Integer log-odds values for OccupiedThreshold
        %   The log-odds value correspond to the default threshold of 0.65
        OccupiedThresholdIntLogodds
    end

    properties (Access = {?occupancyMap, ?nav.algs.internal.InternalAccess,...
                          ?nav.algs.internal.MapUtils, ...
                          ?matlabshared.autonomous.map.internal.InternalAccess})
        %LogoddsHit Integer log-odds value for update on sensor hit
        %   This is the default log-odds value used to update grid cells
        %   where a hit is detected. This value represents probability
        %   value of 0.7
        LogoddsHit

        %LogoddsMiss Integer log-odds value for update on sensor miss
        %   This is the default log-odds value used to update grid cells
        %   where a miss is detected (i.e. laser ray passes through grid
        %   cells). This value represents probability value of 0.4
        LogoddsMiss

        %ProbSatIntLogodds Int log-odds for saturation
        %   These values correspond to default ProbabilitySaturation
        %   values of [0.001 0.999]
        ProbSatIntLogodds
    end
    
    methods %Getter and setter methods for map properties
        function set.OccupiedThreshold(obj,threshold)
        %set.OccupiedThreshold Setter for the limit over which a cell is
        % considered occupied
            freeThreshold = obj.intLogoddsToProb(obj.FreeThresholdIntLogodds);
            validateattributes(threshold, {'numeric'}, ...
                               {'scalar', 'real', 'nonnan', 'finite', 'nonnegative'}, ...
                               'OccupiedThreshold');

            logOddsThreshold = obj.probToIntLogodds(double(threshold));
            check = obj.FreeThresholdIntLogodds > logOddsThreshold;
            if check
                if coder.target('MATLAB')
                    error(message('nav:navalgs:occgrid:ThresholdOutsideBounds', 'OccupiedThreshold', ...
                                  '>=', num2str(freeThreshold,'%.3f')));
                else
                    coder.internal.error('nav:navalgs:occgrid:ThresholdOutsideBounds', 'OccupiedThreshold', ...
                                           '>=', coder.internal.num2str(freeThreshold));
                end
            end
            obj.OccupiedThresholdIntLogodds = logOddsThreshold;
        end

        function threshold = get.OccupiedThreshold(obj)
        %get.OccupiedThreshold Getter for the limit over which a cell is
        % considered occupied
            threshold = obj.intLogoddsToProb(obj.OccupiedThresholdIntLogodds);
        end

        function set.FreeThreshold(obj,threshold)
        %set.FreeThreshold Setter for the limit under which a cell is
        % considered free
            occupiedThreshold = obj.intLogoddsToProb(obj.OccupiedThresholdIntLogodds);
            validateattributes(threshold, {'numeric'}, ...
                               {'scalar', 'real', 'nonnan', 'finite', 'nonnegative'}, ...
                               'FreeThreshold');


            logOddsThreshold = obj.probToIntLogodds(double(threshold));
            check = obj.OccupiedThresholdIntLogodds < logOddsThreshold;
            if check
                if coder.target('MATLAB')
                    error(message('nav:navalgs:occgrid:ThresholdOutsideBounds', 'FreeThreshold', ...
                                  '<=', num2str(occupiedThreshold,'%.3f')));
                else
                    coder.internal.error('nav:navalgs:occgrid:ThresholdOutsideBounds', 'FreeThreshold', ...
                                           '<=', coder.internal.num2str(occupiedThreshold));
                end
            end
            obj.FreeThresholdIntLogodds = logOddsThreshold;
        end

        function threshold = get.FreeThreshold(obj)
        %get.FreeThreshold Getter for the limit under which a cell is
        % considered free
            threshold = obj.intLogoddsToProb(obj.FreeThresholdIntLogodds);
        end

        function set.ProbabilitySaturation(obj, sat)
        %set.ProbabilitySaturation Setter for the probability saturation 
        % thresholds
        
        % Run basic checks
            validateattributes(sat, {'numeric'}, ...
                               {'vector', 'real', 'nonnan', 'numel', 2}, 'ProbabilitySaturation');

            % Check the limits
            sortedSat = sort(sat);

            validateattributes(sortedSat(1), {'numeric'}, {'>=',obj.getMaxSaturation(1), ...
                                '<=',0.5}, ...
                               'ProbabilitySaturation', 'lower saturation');

            validateattributes(sortedSat(2), {'numeric'}, {'>=',0.5, ...
                                '<=',obj.getMaxSaturation(2)}, ...
                               'ProbabilitySaturation', 'upper saturation');

            % Set saturation properties
            obj.ProbSatIntLogodds = obj.probToIntLogodds(sortedSat);
            
            % Update grid if the saturations became tighter
            [I,J] = ind2sub(obj.GridSize,find(obj.getValueGrid < obj.ProbSatIntLogodds(1)));
            obj.setValueGrid([I J], repmat(obj.ProbSatIntLogodds(1),size(I,1),1));
            [I,J] = ind2sub(obj.GridSize,find(obj.getValueGrid > obj.ProbSatIntLogodds(2)));
            obj.setValueGrid([I J], repmat(obj.ProbSatIntLogodds(2),size(I,1),1));
        end

        function sat = get.ProbabilitySaturation(obj)
        %set.ProbabilitySaturation Getter for the probability saturation 
        % thresholds
            sat = obj.intLogoddsToProb(obj.ProbSatIntLogodds);
        end
    end

    methods
        function [occupied, validIds] = checkOccupancy(obj, varargin)
        %checkOccupancy Check ternary occupancy status for one or more positions
        %   MAT = checkOccupancy(MAP) returns an N-by-M matrix, MAT, that
        %   contains the occupancy status of each location based on the
        %   OccupiedThreshold and FreeThreshold properties. Values of 0
        %   refer to obstacle-free cells, 1 refers to occupied cells, and
        %   -1 refers to unknown.
        %
        %   VAL = checkOccupancy(MAP, LOCATIONS) returns an N-by-1 array, VAL, 
        %   that contains the occupancy status for N-by-2 array, LOCATIONS.
        %
        %   VAL = checkOccupancy(MAP, XY, 'world') returns an N-by-1 array,
        %   VAL, that contains the occupancy status for N-by-2 array, XY. Each 
        %   row of XY corresponds to a point with [X Y] world coordinates. 
        %
        %   VAL = checkOccupancy(MAP, XY, 'local') returns an N-by-1 array,
        %   VAL, that contains the occupancy status for N-by-2 array, XY. 
        %   Each row of XY corresponds to a point with [X Y] local coordinates. 
        %
        %   VAL = checkOccupancy(MAP, IJ, 'grid') returns an N-by-1 array,
        %   VAL, that contains the occupancy status for N-by-2 array, IJ. 
        %   Each row of IJ refers to a grid cell index [i j].
        %
        %   [VAL, VALIDPTS] = checkOccupancy(MAP, LOCATIONS, ___ ) returns an
        %   N-by-1 array, VAL, for N-by-2 array, LOCATIONS. If a second
        %   output argument is specified, checkOccupancy also returns an
        %   N-by-1 array of logicals indicating whether input LOCATIONS
        %   are inside map boundaries.
        %
        %   MAT = checkOccupancy(MAP, BOTTOMLEFT, MATSIZE) returns a matrix of
        %   occupancy status in a subregion defined by BOTTOMLEFT and MATSIZE.
        %   By default, BOTTOMLEFT is an [X Y] point in the world frame, and
        %   MATSIZE corresponds to the width and height of the region. XY
        %   locations outside the map limits return -1 for unknown.
        %   
        %   MAT = checkOccupancy(MAP, BOTTOMLEFT, MATSIZE, 'world') returns a 
        %   matrix of occupancy status. BOTTOMLEFT is the bottom left point
        %   of the square region in the world frame, given as [X Y], and 
        %   MATSIZE is the size of the region given as [width, height]
        %
        %   MAT = checkOccupancy(MAP, BOTTOMLEFT, MATSIZE, 'local') returns a 
        %   matrix of occupancy status. BOTTOMLEFT is an [X Y] point in the
        %   local frame, and MATSIZE corresponds to the width and height 
        %   of the region.
        %
        %   MAT = checkOccupancy(MAP, TOPLEFT, MATSIZE, 'grid') returns a 
        %   matrix of occupancy status. TOPLEFT is an [I J] index in the
        %   grid frame, and MATSIZE is a 2-element vector corresponding to 
        %   [rows cols]
        %
        %   Example:
        %       % Create an occupancy grid map.
        %       map = occupancyMap(10, 10);
        %
        %       % Check occupancy status of the world coordinate [0 0].
        %       value = checkOccupancy(map, [0 0]);
        %
        %       % Check occupancy status of multiple coordinates.
        %       [X, Y] = meshgrid(0:0.5:5);
        %       values = checkOccupancy(map, [X(:) Y(:)]);
        %
        %       % Check occupancy status of the grid cell [1 1].
        %       value = checkOccupancy(map, [1 1], 'grid');
        %
        %       % Check occupancy status of multiple grid cells.
        %       [I, J] = meshgrid(1:5);
        %       values = checkOccupancy(map, [I(:) J(:)], 'grid');
        %
        %   See also occupancyMap, getOccupancy
            
            if nargin == 1
                value = obj.getValueAllImpl;
                
                freeIdx = (value < obj.FreeThresholdIntLogodds);
                occIdx = (value > obj.OccupiedThresholdIntLogodds);
                
                occupied = repmat(-1,size(value));
                occupied(freeIdx) = 0;
                occupied(occIdx) = 1;
            else
                [ptIsValid, matSize, isGrid, isLocal] = obj.getParser('checkOccupancy', varargin{:});
                
                occupied = obj.checkOccupancyImpl(varargin{1}, matSize, isGrid, isLocal);
            end
            
            if nargout == 2
                % If second output argument is requested, return N-by-1
                % vector of logicals corresponding to the vector of xy or
                % ij points. Any points that lie outside of boundaries, or
                % that contain nan or non-finite coordinates return false.
                validIds = ptIsValid;
            end
        end
        
        function cpObj = copy(obj)
        %COPY Create a copy of the object
        %   cpObj = COPY(obj) creates a deep copy of the
        %   Occupancy Grid object with the same properties.
        %
        %   Example:
        %       % Create a 10x10 occupancy map.
        %       map = occupancyMap(10, 10);
        %
        %       % Create a copy of the object.
        %       cpObj = copy(map);
        %
        %       % Access the class methods from the new object.
        %       setOccupancy(cpObj,[2 4],true);
        %
        %       % Delete the handle object.
        %       delete(cpObj)
        %
        %   See also occupancyMap

            if isempty(obj)
                % This will not be encountered in code generation
                cpObj = occupancyMap.empty(0,1);
            else
                % Create a new object with the same properties
                if isstruct(obj.SharedProperties)
                    obj.SharedProperties = matlabshared.autonomous.internal.SharedMapProperties(...
                        obj.SharedProperties);
                    cpObj = copy(obj);
                else
                    cpObj = occupancyMap(obj);
                    cpObj.SharedProperties = copy(obj.SharedProperties);
                end
            end
        end
        
        function [value, validIds] = getOccupancy(obj, varargin)
        %getOccupancy Get occupancy value for one or more positions
        %   MAT = getOccupancy(MAP) returns an N-by-M matrix of occupancy
        %   values.
        %   
        %   VAL = getOccupancy(MAP, LOCATIONS) returns an N-by-1 array of
        %   occupancy values for N-by-2 array, LOCATIONS. Locations found
        %   outside the bounds of the map return map.DefaultValue.
        %
        %   VAL = getOccupancy(MAP, XY, 'world') returns an N-by-1 array of
        %   occupancy values for N-by-2 array XY in world coordinates.
        %   This is the default reference frame.
        %
        %   VAL = getOccupancy(MAP, XY, 'local') returns an N-by-1 array of
        %   occupancy values for N-by-2 array, XY. Each row of the
        %   array XY corresponds to a point with [X Y] local coordinates.
        %
        %   VAL = getOccupancy(MAP, IJ, 'grid') returns an N-by-1
        %   array of occupancy values for N-by-2 array IJ. Each row of
        %   the array IJ refers to a grid cell index [i j].
        %
        %   [VAL, VALIDPTS] = getOccupancy(MAP, LOCATIONS, ___ ) returns an
        %   N-by-1 array of occupancy values for N-by-2 array, LOCATIONS. If 
        %   a second output argument is specified, getOccupancy also returns
        %   an N-by-1 vector of logicals indicating whether input LOCATIONS
        %   are inside map boundaries.
        %
        %   MAT = getOccupancy(MAP, BOTTOMLEFT, MATSIZE) returns a matrix of
        %   occupancy values in a subregion defined by BOTTOMLEFT and MATSIZE.
        %   BOTTOMLEFT is the bottom left point of the square region in the
        %   world frame, given as [X Y], and MATSIZE is the size of the
        %   region given as [width, height].
        %   
        %   MAT = getOccupancy(MAP, BOTTOMLEFT, MATSIZE, 'world') returns a 
        %   matrix of occupancy values. BOTTOMLEFT is an [X Y] point in the
        %   world frame, and MATSIZE corresponds to the width and height 
        %   of the region.
        %
        %   MAT = getOccupancy(MAP, BOTTOMLEFT, MATSIZE, 'local') returns a 
        %   matrix of occupancy values. BOTTOMLEFT is an [X Y] point in the
        %   local frame, and MATSIZE corresponds to the width and height 
        %   of the region.
        %
        %   MAT = getOccupancy(MAP, TOPLEFT, MATSIZE, 'grid') returns a 
        %   matrix of occupancy values. TOPLEFT is an [I J] index in the
        %   grid frame, and MATSIZE is a 2-element vector corresponding to 
        %   [rows cols]
        %
        %   Example:
        %       % Create an occupancy map and get occupancy values for a 
        %       % position relative to the world frame.
        %       map = occupancyMap(10, 10);
        %
        %       % Get occupancy of the world coordinate [0 0].
        %       value = getOccupancy(map, [0 0]);
        %
        %       % Get occupancy of multiple coordinates.
        %       [X, Y] = meshgrid(0:0.5:5);
        %       values = getOccupancy(map, [X(:) Y(:)]);
        %
        %       % Get occupancy of the grid cell [1 1].
        %       value = getOccupancy(map, [1 1], 'grid');
        %
        %       % Get occupancy of multiple grid cells.
        %       [I, J] = meshgrid(1:5);
        %       values = getOccupancy(map, [I(:) J(:)], 'grid');
        %
        %   See also occupancyMap, setOccupancy
            
            if nargout == 2
                % If second output argument is requested, return N-by-1
                % vector of logicals corresponding to the vector of xy or
                % ij points. Any points that lie outside of boundaries, or
                % that contain nan or non-finite coordinates return false.
                [value, validIds] = obj.getMapData(varargin{:});
            else
                value = obj.getMapData(varargin{:});
            end
        end
        
        function inflate(obj, varargin)
        %INFLATE Inflate occupied positions by a given amount
        %   INFLATE(MAP, R) inflates each occupied position of the
        %   occupancy grid map by at least R meters. Each cell of the
        %   occupancy grid map is inflated by number of cells which is the
        %   closest integer higher than the value MAP.Resolution*R.
        %
        %   INFLATE(MAP, R, 'grid') inflates each cell of the
        %   occupancy grid map by R cells.
        %
        %   Note that the inflate function does not inflate the
        %   positions past the limits of the grid.
        %
        %   Example:
        %       % Create an occupancy map and inflate map.
        %       mat = eye(100)*0.6;
        %       map = occupancyMap(mat);
        %
        %       % Create a copy of the map for inflation.
        %       cpMap = copy(map);
        %
        %       % Inflate occupied cells using inflation radius in meters.
        %       inflate(cpMap, 0.1);
        %
        %       % Inflate occupied cells using inflation radius in number 
        %       % of cells.
        %       inflate(cpMap, 2, 'grid');
        %
        %   See also occupancyMap, copy

            narginchk(2,3);
            inflatedGrid = nav.algs.internal.MapUtils.inflateGrid(...
                obj, obj.getValueAllImpl, varargin{:});
            obj.setValueMatrixImpl(inflatedGrid);
        end

        function move(obj, moveValue, varargin)
        %MOVE Move map in world frame
        %   MOVE(MAP, MOVEVALUE) moves the local origin of MAP to a location, 
        %   MOVEVALUE, given as an [x y] vector and updates the map limits.
        %   The MOVEVALUE is truncated based on the resolution of the map. 
        %   Values at locations within the previous limits and map limits.
        %   
        %   MOVE(MAP,___,Name,Value) provides additional options specified
        %   by one or more Name,Value pair arguments. Name must appear
        %   inside single quotes (''). You can specify several name-value
        %   pair arguments in any order as Name1,Value1,...,NameN,ValueN:
        %
        %       'MoveType'      - A string that modifies the meaning of
        %                         MOVEVALUE:
        %                          
        %                             'Absolute' (default) - MAP moves its
        %                             local origin to discretized [x y] world 
        %                             frame position.
        %
        %                             'Relative' - MAP translates by
        %                             discrete [x y] distance, relative to
        %                             its original location in world frame.
        %
        %       'FillValue'     - A scalar value, FILLVALUE, for filling
        %                         unset locations. New locations that fall
        %                         outside the original map frame are
        %                         initialized with FILLVALUE.
        %
        %       'SyncWith'      - An occupancyMap object, SOURCEMAP,
        %                         for syncing unset locations. New locations
        %                         outside the original map frame that exist
        %                         in SOURCEMAP are initialized with values
        %                         in SOURCEMAP. New locations outside of
        %                         SOURCEMAP are set with the DefaultValue
        %                         property or specified FILLVALUE.
        %
        %   Example:
        %       % Create an occupancyMap.
        %       map = occupancyMap(eye(20),2);
        %
        %       % Try to move the map to [1.25 3.75].
        %       move(map, [1.25 3.75])
        %
        %           % NOTE: Resolution = 2, so the map moves to discrete
        %           % world location [1 3.5]
        %
        %       % Translate the map by XY distance [5 5] relative to world 
        %       % frame.
        %       move(map, [5 5], 'MoveType', 'Relative')
        %
        %       % Translate the map by [0 3] and fill new locations with a
        %       % value of 0.75.
        %       move(map, [0 3], 'MoveType', 'Relative', 'FillValue', .75)
        %
        %       % Move the map back to [0 0] and fill new locations with
        %       % data from another map where the frames overlap.
        %       worldMap = occupancyMap(rand(100,200))
        %       move(map, [0 0], 'SyncWith', worldMap)
        %
        %       % Move the map to [-1 -1], and fill new locations with
        %       % data from another map. Fill all other new locations that
        %       % do not overlap with a value of 0.5.
        %       move(map, [-1 -1], 'SyncWith', worldMap, 'FillValue', .5)
        %
        %   See also binaryOccupancyMap

            % Parse inputs
            [moveValue, syncMap, fillVal] = obj.moveParser(moveValue, varargin{:});
                                          
            % Call internal move command
            move@matlabshared.autonomous.internal.MapLayer(obj, moveValue, ...
                            'DownSamplePolicy','Max','FillWith',fillVal,'SyncWith',syncMap);
        end
        
        function mat = occupancyMatrix(obj, option)
        %OCCUPANCYMATRIX Export occupancyMap as a matrix
        %   MAT = OCCUPANCYMATRIX(MAP) returns occupancy values stored 
        %   in the occupancyMap object as a double matrix of size GridSize.
        %
        %   MAT = OCCUPANCYMATRIX(MAP, 'ternary') returns occupancy status 
        %   of each occupancy grid cell as a matrix. The OccupiedThreshold
        %   and FreeThreshold are used to determine obstacle-free and 
        %   occupied cells. Value 0 refers to obstacle-free cell, 1 refers
        %   to occupied cell and -1 refers to unknown cell.
        %
        %   Example:
        %       % Create an occupancy grid map.
        %       inputMat = repmat(0.2:0.1:0.9, 8, 1);
        %       map = occupancyMap(inputMat);
        %
        %       % Export occupancy grid map as a matrix.
        %       mat = occupancyMatrix(map);
        %
        %       % Export occupancy grid map as a ternary matrix.
        %       mat = occupancyMatrix(map, 'ternary');
        %
        %   See also occupancyMap, getOccupancy

            narginchk(1,2);
            if nargin < 2
                mat = obj.intLogoddsToProb(obj.getValueAllImpl);
                return;
            end

            validStrings = {'Ternary'};
            validatestring(option, validStrings, 'occupancyMatrix');
            gSize = obj.SharedProperties.GridSize;
            mat = repmat(-1,gSize(1),gSize(2));

            occupied = (obj.getValueAllImpl > obj.OccupiedThresholdIntLogodds);
            free = (obj.getValueAllImpl < obj.FreeThresholdIntLogodds);
            mat(occupied) = 1;
            mat(free) = 0;
        end

        function validIds = setOccupancy(obj, varargin)
        %setOccupancy Set occupancy value for one or more locations
        %   setOccupancy(MAP, INPUTMATRIX) sets the MAP occupancy values to
        %   the values in INPUTMATRIX. The matrix must be the same size as
        %   GridSize property of MAP.
        %
        %   setOccupancy(MAP, LOCATIONS, VAL) assigns each element of the
        %   N-by-1 vector, VAL to the coordinate position of the
        %   corresponding row of the N-by-2 array, LOCATIONS. Locations
        %   found outside map boundaries are ignored.
        %
        %   setOccupancy(MAP, XY, VAL, 'world') assigns the occupancy
        %   values of the N-element array, VAL, to each location specified 
        %   in the N-by-2 [x y] matrix, XY. Locations are specified in
        %   world coordinates. This is the default reference frame.
        %
        %   setOccupancy(MAP, XY, VAL, 'local') assigns the occupancy
        %   values of the N-element array, VAL, to each location specified 
        %   in the N-by-2 [x y] matrix, XY. Locations are specified in
        %   local coordinates.
        %
        %   setOccupancy(MAP, IJ, VAL, 'grid') assigns the occupancy
        %   values of the N-element array, VAL, to each cell specified 
        %   in the N-by-2 [i j] matrix, IJ.
        %
        %   VALIDPTS = setOccupancy(MAP, LOCATIONS, VAL, ___ ) assigns each
        %   element of the N-by-1 vector, VAL to the coordinate position of the
        %   corresponding row of the N-by-2 array, LOCATIONS. If an output
        %   argument is specified, setOccupancy returns an N-by-1 vector of
        %   logicals indicating whether the input points were inside the
        %   map boundaries.
        %
        %   setOccupancy(MAP, BOTTOMLEFT, INPUTMAT) assigns an N-by-M occupancy
        %   matrix, INPUTMAT, to the MAP. The subregion begins in the [I J]
        %   cell corresponding to [X Y] world position, BOTTOMLEFT, and
        %   extends [N M] rows/cols in the -I +J direction.
        %   
        %   setOccupancy(MAP, BOTTOMLEFT, INPUTMAT, 'world') assigns an 
        %   N-by-M occupancy matrix, INPUTMAT, to the MAP. The subregion
        %   begins in the [I J] cell corresponding to [X Y] world position,
        %   BOTTOMLEFT, and extends [N M] rows/cols in the -I +J direction.
        %
        %   setOccupancy(MAP, BOTTOMLEFT, INPUTMAT, 'local') assigns an 
        %   N-by-M occupancy matrix, INPUTMAT, to the MAP. The subregion
        %   begins in the [I J] cell corresponding to [X Y] local position,
        %   BOTTOMLEFT, and extends [N M] rows/cols in the -I +J direction.
        %
        %   setOccupancy(MAP, TOPLEFT, INPUTMAT, 'grid') assigns an 
        %   N-by-M occupancy matrix, INPUTMAT, to the MAP. The subregion
        %   begins in the [I J] cell, TOPLEFT, and extends [N M] rows/cols
        %   in the +I +J direction.
        %
        %   Example:
        %       % Create a map and set occupancy values for a position.
        %       map = occupancyMap(10, 10);
        %
        %       % Set occupancy of the world coordinate [0 0] to 0.75.
        %       setOccupancy(map, [0 0], .75);
        %
        %       % Set occupancy of multiple coordinates.
        %       [X, Y] = meshgrid(0:0.5:5);
        %       values = rand(numel(X),1);
        %       setOccupancy(map, [X(:) Y(:)], values);
        %
        %       % Set occupancy of the grid cell [1 1] to 0.25.
        %       setOccupancy(map, [1 1], .25, 'grid');
        %
        %       % Set occupancy of multiple grid cells to the same value
        %       [I, J] = meshgrid(1:5);
        %       setOccupancy(map, [I(:) J(:)], 1, 'grid');
        %
        %       % Set occupancy of multiple grid cells.
        %       [I, J] = meshgrid(1:5);
        %       values = rand(numel(I), 1);
        %       setOccupancy(map, [I(:) J(:)], values, 'grid');
        %
        %   See also occupancyMap, getOccupancy
            
            if nargout == 1
                % If output argument is requested, return N-by-1
                % vector of logicals corresponding to the vector of xy or
                % ij points. Any points that lie outside of boundaries, or
                % that contain nan or non-finite coordinates return false.
                validIds = obj.setMapData(varargin{:});
            else
                obj.setMapData(varargin{:});
            end
        end

        function imageHandle = show(obj, varargin)
        %SHOW Display the occupancy map in a figure
        %   SHOW(MAP) displays the occupancyMap object, MAP, in the current
        %   axes with the axes labels representing the world coordinates.
        %
        %   SHOW(MAP, 'local') displays the occupancyMap object, MAP, the
        %   in the current axes with the axes of the figure representing
        %   local coordinates of the MAP. The default input is 'world',
        %   which shows the axes in world coordinates.
        %
        %   SHOW(MAP, 'grid') displays the occupancyMap object, MAP, in the 
        %   current axes with the axes of the figure representing the grid
        %   indices.
        %
        %   HIMAGE = SHOW(MAP, ___) returns the handle to the image object 
        %   created by show.
        %
        %   SHOW(MAP,___,Name,Value) provides additional options specified
        %   by one or more Name,Value pair arguments. Name must appear
        %   inside single quotes (''). You can specify several name-value
        %   pair arguments in any order as Name1,Value1,...,NameN,ValueN:
        %
        %       'Parent'        - Axes to plot the map, specified as an axes handle.
        %
        %                         Default: gca
        %
        %       'FastUpdate'    - Boolean value used to speed up show method
        %                         for existing map plots. If you have 
        %                         previously plotted your map on the axes,
        %                         specify 1 to perform a lightweight update
        %                         to the map in the figure.
        %
        %                         Default: 0 (regular update)
        %
        %   Example:
        %       % Create a map.
        %       map = occupancyMap(eye(5));
        %
        %       % Display the occupancy with axes showing the world
        %       % coordinates.
        %       gh = show(map);
        %
        %       % Display the occupancy with axes showing the grid indices.
        %       gh = show(map, 'grid');
        %
        %       % Display the occupancy with axes showing the world
        %       % coordinates and specify a parent axes.
        %       fh = figure;
        %       ah = axes('Parent', fh);
        %       gh = show(map, 'world', 'Parent', ah);
        %
        %   See also occupancyMap

            [axHandle, isGrid, isLocal, fastUpdate] = ...
                nav.algs.internal.MapUtils.showInputParser(varargin{:});
            [axHandle, imghandle, fastUpdate] = ...
                nav.algs.internal.MapUtils.showGrid(obj, axHandle, isGrid, isLocal, fastUpdate);
            
            if ~fastUpdate
                axHandle.Title.String = obj.ShowTitle;
            end
            
            % Only return handle if user requested it.
            if nargout > 0
                imageHandle = imghandle;
            end
        end
        
        function syncWith(obj, sourceMap)
        %SYNCWITH Sync map with overlapping map
        %
        %   syncWith(MAP,SOURCEMAP) updates MAP with data from another
        %   occupancyMap, SOURCEMAP. Locations in MAP that are also
        %   found in SOURCEMAP are updated, all other cells retain their
        %   current values. 
        %
        %   Example:
        %       % Set a localMap's initial position and sync its data with
        %       % a world map.
        %
        %       % Create a 100x100 world map.
        %       worldMap = occupancyMap(eye(100));
        %       
        %       % Create a 10x10 local map.
        %       localMap = occupancyMap(10,10);
        %
        %       % Set the localMap's local-frame to [45 45].
        %       localMap.LocalOriginInWorld = [45 45];
        %
        %       % Sync localMap's data with worldMap.
        %       syncWith(localMap, worldMap)
        %
        %       % Set localMap's initial position partially outside the
        %       % worldMap limits and sync.
        %       localMap.LocalOriginInWorld = [-5 -5];
        %       syncWith(localMap, worldMap);
        %
        %   See also binaryOccupancyMap, move
            
            % Validate source-map type
            validateattributes(sourceMap,{'occupancyMap'},{'scalar'},'syncWith','sourceMap');
        
            % Write data from overlapping region of sourceMap into map
            obj.writeFromOtherMap(sourceMap);
        end
        
        function updateOccupancy(obj, varargin)
        %updateOccupancy Integrate occupancy value for one or more positions
        %   updateOccupancy(MAP, INPUTMATRIX) probabilistically integrates
        %   the INPUTMATRIX with MAP's current occupancy matrix. The size
        %   of the matrix must be equal to the GridSize property. Default 
        %   update values are used if INPUTMATRIX is logical. Update values
        %   are 0.7 and 0.4 for true and false respectively. Alternatively,
        %   INPUT MATRIX can be of any numeric type with value between 0 and 1.
        %
        %   updateOccupancy(MAP, LOCATIONS, OBS) integrates observation 
        %   values, OBS, into the cells corresponding to the N-by-2 array, 
        %   LOCATIONS. OBS can be a scalar that is applied to every location, 
        %   or an N-element vector. Locations found outside map boundaries 
        %   are ignored.
        %
        %   updateOccupancy(MAP, XY, OBS, 'world') integrates observation
        %   values, OBS, into cells found at the N-by-2 set of world coordinates, XY.
        %
        %   updateOccupancy(MAP, XY, OBS, 'local') integrates observation
        %   values, OBS, into cells found at the N-by-2 set of local coordinates, XY.
        %
        %   updateOccupancy(MAP, IJ, OBS, 'grid') integrates observation
        %   values, OBS, into cells specified by the N-by-2 set of indices, IJ.
        %
        %   updateOccupancy(MAP, BOTTOMLEFT, INPUTMAT) integrates an N-by-M
        %   matrix of observations, INPUTMAT, to the MAP. The subregion 
        %   begins in the bottom-left [X Y] world position, BOTTOMLEFT, and extends 
        %   N rows up and M columns to the right.
        %   
        %   updateOccupancy(MAP, BOTTOMLEFT, INPUTMAT, 'world') integrates an 
        %   N-by-M matrix of observations, INPUTMAT, to the MAP. The subregion 
        %   begins in the bottom-left [X Y] world position, BOTTOMLEFT, and  
        %   extends N rows up and M columns to the right.
        %
        %   updateOccupancy(MAP, BOTTOMLEFT, INPUTMAT, 'local') assigns an 
        %   N-by-M matrix of observations, INPUTMAT, to the MAP. The subregion 
        %   begins in the bottom-left [X Y] local position, BOTTOMLEFT, and  
        %   extends N rows up and M columns to the right.
        %
        %   updateOccupancy(MAP, TOPLEFT, INPUTMAT, 'grid') assigns an 
        %   N-by-M matrix of observations, INPUTMAT, to the MAP. The subregion 
        %   begins in the top-left [I J] cell, TOPLEFT, and extends N rows 
        %   down and M columns to the right.
        %
        %   Example:
        %       % Create an occupancy map and update occupancy values for a
        %       % position.
        %       map = occupancyMap(10, 10);
        %
        %       % Update occupancy of the world coordinate [0 0].
        %       updateOccupancy(map, [0 0], true);
        %
        %       % Update occupancy of multiple coordinates.
        %       [X, Y] = meshgrid(0:0.5:5);
        %       values = ones(numel(X),1)*0.65;
        %       updateOccupancy(map, [X(:) Y(:)], values);
        %
        %       % Update occupancy of the grid cell [1 1].
        %       updateOccupancy(map, [1 1], false, 'grid');
        %
        %       % Update occupancy of multiple grid cells.
        %       [I, J] = meshgrid(1:5);
        %       updateOccupancy(map, [I(:) J(:)], 0.4, 'grid');
        %
        %       % Update occupancy of multiple grid cells.
        %       [I, J] = meshgrid(1:5);
        %       values = true(numel(I),1);
        %       updateOccupancy(map, [I(:) J(:)], values, 'grid');
        %
        %   See also occupancyMap, setOccupancy
            
            narginchk(2,5);
        
            % Determine which syntax is being used
            [~, ~, isMat, isGrid, isLocal] = obj.setParser('updateOccupancy', varargin{:});
        
            if isMat
                % Update a region of the map with occupancy observations
                obj.updateRegion(isGrid, isLocal, varargin{:});
            else
                % Update one or more cells with occupancy observations
                obj.updateIndices(isGrid,isLocal,varargin{1},varargin{2});
            end
        end
    end %General methods

    methods %raycasting methods
        function insertRay(obj, varargin)
        %insertRay Insert rays from laser scan observation
        %   insertRay(MAP, POSE, SCAN, MAXRANGE) inserts range sensor
        %   readings in the occupancy grid map. POSE is a 3-element vector
        %   representing sensor pose [X, Y, THETA] in the world coordinate
        %   frame, SCAN is a scalar lidarScan object with range sensor
        %   readings, and MAXRANGE is the maximum range of the sensor.
        %   The cells along the ray except the end points are observed
        %   as obstacle-free and updated with probability of 0.4.
        %   The cells touching the end point are observed as occupied
        %   and updated with probability of 0.7. NaN values in the SCAN
        %   ranges are ignored. SCAN ranges above MAXRANGE are truncated
        %   and the end points are not updated for MAXRANGE readings.
        %
        %   insertRay(MAP, POSE, RANGES, ANGLES, MAXRANGE) allows
        %   you to pass range sensor readings as RANGES and ANGLES. The 
        %   input ANGLES are in radians.
        %
        %   insertRay(MAP, POSE, SCAN, MAXRANGE, INVMODEL)
        %   inserts the range sensor readings, SCAN, with update
        %   probabilities according to a 2-element vector INVMODEL.
        %   The first element of INVMODEL is used to update obstacle-free
        %   observations and the second element is used to update occupied
        %   observations. Values in INVMODEL should be between 0 and 1.
        %
        %   insertRay(MAP, POSE, RANGES, ANGLES, MAXRANGE, INVMODEL)
        %   allows you to pass range sensor readings as RANGES and ANGLES.
        %
        %   insertRay(MAP, STARTPT, ENDPTS) inserts cells between the
        %   line segments STARTPT and ENDPTS. STARTPT is 2-element vector
        %   representing the start point [X,Y] in the world coordinate frame.
        %   ENDPTS is N-by-2 array of end points in the world coordinate
        %   frame. The cells along the line segment except the end points
        %   are updated with miss probability of 0.4 and the cells
        %   touching the end point are updated with hit probability of 0.7.
        %
        %   insertRay(MAP, STARTPT, ENDPTS, INVMODEL) inserts the
        %   line segment with update probabilities according to a
        %   1-by-2 or 2-by-1 array INVMODEL.
        %
        %   Example:
        %       % Create a map
        %       map = occupancyMap(10,10,20);
        %
        %       % Insert two rays
        %       scan = lidarScan([5, 6], [pi/4, pi/6]);
        %       insertRay(map, [5,5,0], scan, 20);
        %
        %       % Insert rays with non default inverse model
        %       insertRay(map, [5,5,0], [5, 6], [pi/3, pi/2], 20, [0.3 0.8]);
        %
        %       % Visualize inserted rays
        %       show(map);
        %
        %       % Insert a line segment
        %       insertRay(map, [0,0], [3,3]);
        %
        %       % Visualize inserted ray
        %       show(map);
        %
        %   See also occupancyMap, raycast
            
            % Parse inputs to function
            [startPt, endPt, inverseModelLogodds, maxRange, rangeIsMax] = ...
                nav.algs.internal.MapUtils.parseInsertRayInputs(obj, [obj.LogoddsMiss obj.LogoddsHit], varargin{:});
            
            numRays = size(endPt,1);
            gSize     = obj.SharedProperties.GridSize;
            res       = obj.SharedProperties.Resolution;
            gLocWorld = obj.SharedProperties.GridLocationInWorld;
            
            % Process rays in batches. Attempts to limit the size of allocated 
            % midpoint array <= 10MB. If a single ray is large enough to
            % exceed 10MB, insertRay processes the rays individually. See
            % raycastInternal for memory allocation rationale. To be 
            % conservative, the calculation assumes indices are stored as
            % 8-byte doubles.
            maxRaysPerIteration = ceil((10*1024*1024)/((4+3*maxRange*res)*2*8));
            
            if numRays == 0
                return;
            else
                startIdx = 1;
                while startIdx <= numRays
                    endIdx = min(numRays,startIdx+maxRaysPerIteration);
                    
                    % Get all points affected by raycast
                    if ~isempty(rangeIsMax)
                        [endPts, middlePts] = nav.algs.internal.raycastCells(startPt, endPt(startIdx:endIdx,:), ...
                                gSize(1), gSize(2), res, gLocWorld, rangeIsMax(startIdx:endIdx));
                    else
                        [endPts, middlePts] = nav.algs.internal.raycastCells(startPt, endPt(startIdx:endIdx,:), ...
                                gSize(1), gSize(2), res, gLocWorld);
                    end
                    
                    if ~isempty(middlePts)
                        % To ensure overlaps are updated properly, update the value
                        % based on the number of occurrences found
                        linIdxMid = middlePts(:,1) + gSize(1)*(middlePts(:,2)-1);
                        [~, idxMid, occurrenceMid] = unique(linIdxMid);
                        
                        numRepeat = int16(accumarray(occurrenceMid,1));
                        updateValuesMiss = obj.getValueAtIndicesInternal(middlePts(idxMid,:)) + numRepeat*inverseModelLogodds(1);
                        updateValuesMiss(updateValuesMiss < obj.ProbSatIntLogodds(1)) = obj.ProbSatIntLogodds(1);
                        obj.setValueAtIndicesInternal(middlePts(idxMid,:), updateValuesMiss);
                    end
                    if ~isempty(endPts)
                        % To ensure overlaps are updated properly, update the value
                        % based on the number of occurrences found
                        linIdxEnd = endPts(:,1) + gSize(1)*(endPts(:,2)-1);
                        [~, idxEnd, occurrenceEnd] = unique(linIdxEnd);
                        
                        numRepeat = int16(accumarray(occurrenceEnd,1));
                        updateValuesHit = obj.getValueAtIndicesInternal(endPts(idxEnd,:)) + numRepeat*inverseModelLogodds(2);
                        updateValuesHit(updateValuesHit > obj.ProbSatIntLogodds(2)) = obj.ProbSatIntLogodds(2);
                        obj.setValueAtIndicesInternal(endPts(idxEnd,:), updateValuesHit);
                    end
                    startIdx = endIdx+1;
                end
            end
        end

        function [endPts, middlePts] = raycast(obj, varargin)
        %RAYCAST Compute cell indices along a ray
        %   [ENDPTS, MIDPTS] = RAYCAST(MAP, POSE, RANGE, ANGLE) returns
        %   cell indices of all cells traversed by a ray emanating from
        %   POSE at an angle ANGLE with length equal to RANGE. POSE is
        %   a 3-element vector representing robot pose [X, Y, THETA] in
        %   the world coordinate frame. ANGLE and RANGE are scalars.
        %   The ENDPTS are indices of cells touched by the
        %   end point of the ray. MIDPTS are all the cells touched by
        %   the ray excluding the ENDPTS.
        %
        %   [ENDPTS, MIDPTS] = RAYCAST(MAP, P1, P2) returns
        %   the cell indices of all cells between the line segment
        %   P1=[X1,Y1] to P2=[X2,Y2] in the world coordinate frame.
        %
        %   For faster insertion of range sensor data, use the insertRay
        %   method with an array of ranges or an array of end points.
        %
        %   Example:
        %       % Create an occupancy grid map
        %       map = occupancyMap(10, 10, 20);
        %
        %       % compute cells along a ray
        %       [endPts, midPts] = raycast(map, [5,3,0], 4, pi/3);
        %
        %       % Change occupancy cells to visualize
        %       updateOccupancy(map, endPts, true, 'grid');
        %       updateOccupancy(map, midPts, false, 'grid');
        %
        %       % Compute cells along a line segment
        %       [endPts, midPts] = raycast(map, [2,5], [6,8]);
        %
        %       % Change occupancy cells to visualize
        %       updateOccupancy(map, endPts, true, 'grid');
        %       updateOccupancy(map, midPts, false, 'grid');
        %
        %       % Visualize the raycast output
        %       show(map);
        %
        %   See also occupancyMap, insertRay

            [endPts, middlePts] = nav.algs.internal.MapUtils.raycast(obj, varargin{:});
        end

        function collisionPt = rayIntersection(obj, pose, angles, maxRange, threshold)
        %rayIntersection Compute map intersection points of rays
        %   PTS = rayIntersection(MAP, POSE, ANGLES, MAXRANGE) returns
        %   collision points PTS in the world coordinate frame for
        %   rays emanating from POSE. PTS is an N-by-2 array of points.
        %   POSE is a 1-by-3 array of sensor pose [X Y THETA] in the world
        %   coordinate frame. ANGLES is an N-element vector of angles
        %   at which to get ray intersection points. MAXRANGE is a
        %   scalar representing the maximum range of the range sensor. If
        %   there is no collision up-to the maximum range then [NaN NaN]
        %   output is returned. By default, the OccupiedThreshold
        %   property in MAP is used to determine occupied cells.
        %
        %   PTS = rayIntersection(MAP, POSE, ANGLES, MAXRANGE, THRESHOLD)
        %   returns collision points PTS, where the THRESHOLD is used
        %   to determine the occupied cells. Any cell with a probability
        %   value greater or equal to THRESHOLD is considered occupied.
        %
        %   Example:
        %       % Create an occupancy grid map
        %       map = occupancyMap(eye(10));
        %
        %       % Set occupancy of the world coordinate (5, 5)
        %       setOccupancy(map, [5 5], 0.5);
        %
        %       % Get collision points
        %       collisionPts = rayIntersection(map, [0,0,0], [pi/4, pi/6], 10);
        %
        %       % Visualize the collision points
        %       show(map);
        %       hold on;
        %       plot(collisionPts(:,1),collisionPts(:,2) , '*')
        %
        %       % Get collision points with threshold value 0.4
        %       collisionPts = rayIntersection(map, [0,0,0], [pi/4, pi/6], 10, 0.4);
        %
        %       % Visualize the collision points
        %       plot(collisionPts(:,1),collisionPts(:,2) , '*')
        %       hold off;
        %
        %   See also occupancyMap, raycast

            narginchk(4,5);

            if nargin < 5
                grid = (obj.occupancyMatrix('ternary') == 1);
            else
                validateattributes(threshold, {'numeric'}, ...
                                   {'real', 'nonnan', 'finite', 'scalar', '<=', 1, '>=', 0}, 'rayIntersection', 'threshold');
                grid = obj.occupancyMatrix > threshold;
            end

            collisionPt = nav.algs.internal.MapUtils.rayIntersection(obj,...
                grid, pose, angles, maxRange);
        end
    end
    
    methods (Hidden)
        function [value, validIds] = getMapData(obj, varargin)
            if nargin == 1
                value = obj.intLogoddsToProb(obj.getValueAllImpl);
            else
                [ptIsValid, matSize, isGrid, isLocal] = obj.getParser('getOccupancy', varargin{:});

                if isempty(matSize)
                    % Individual points
                    if isGrid
                        value = obj.intLogoddsToProb(obj.getValueAtIndicesInternal(varargin{1}));
                    elseif isLocal
                        value = obj.intLogoddsToProb(obj.getValueLocalAtIndicesImpl(varargin{1}));
                    else
                        value = obj.intLogoddsToProb(obj.getValueWorldAtIndicesImpl(varargin{1}));
                    end
                else
                    % Block syntax
                    if isGrid
                        value = obj.intLogoddsToProb(obj.getValueGridBlockImpl(varargin{1}, matSize(1), matSize(2)));
                    elseif isLocal
                        value = obj.intLogoddsToProb(obj.getValueLocalBlockImpl(varargin{1}, matSize(1), matSize(2)));
                    else
                        value = obj.intLogoddsToProb(obj.getValueWorldBlockImpl(varargin{1}, matSize(1), matSize(2)));
                    end
                end
            end

            if nargout == 2
                % If second output argument is requested, return N-by-1
                % vector of logicals corresponding to the vector of xy or
                % ij points. Any points that lie outside of boundaries, or
                % that contain nan or non-finite coordinates return false.
                validIds = ptIsValid;
            end
        end
        
        function validIds = setMapData(obj, varargin)
            [vals, ptIsValid, isMat, isGrid, isLocal] = obj.setParser('setOccupancy', varargin{:});
            
            if nargin == 2
                obj.setValueMatrixImpl(probToIntLogodds(obj,double(vals)));
            elseif ~isMat
                nav.algs.internal.MapUtils.validateOccupancyValues(vals(:), size(varargin{1}, 1), 'setOccupancy', 'VAL', 2);
                % Individual points
                if isGrid
                    obj.setValueAtIndicesInternal(varargin{1}, probToIntLogodds(obj,double(vals(:))));
                elseif isLocal
                    obj.setValueAtIndicesLocalImpl(varargin{1}, probToIntLogodds(obj,double(vals(:))));
                else
                    obj.setValueAtIndicesWorldImpl(varargin{1}, probToIntLogodds(obj,double(vals(:))));
                end
            else
                % Block syntax
                if isGrid
                    obj.setBlockInternal(varargin{1}, probToIntLogodds(obj,double(vals)));
                elseif isLocal
                    obj.setValueLocalBlockImpl(varargin{1}, probToIntLogodds(obj,double(vals)));
                else
                    obj.setValueWorldBlockImpl(varargin{1}, probToIntLogodds(obj,double(vals)));
                end
            end
            
            if nargout == 1
                validIds = ptIsValid;
            end
        end
    end
    
    methods (Static)
        function obj = loadobj(s)
        %loadobj Load saved occupancyMap
            if (isstruct(s))
                if isfield(s,'XWorldLimits')
                    width  = diff(s.XWorldLimits);
                    height = diff(s.YWorldLimits);
                else
                    width  = s.GridSize(2)/s.Resolution;
                    height = s.GridSize(1)/s.Resolution;
                end
                obj = occupancyMap(width, height, s.Resolution);
                obj.setValueMatrixImpl(s.Logodds);
                fields = fieldnames(s);
                for i = 1:length(fields)
                    obj.(fields{i}) = s.(fields{i});
                end
            elseif (isa(s,'occupancyMap'))
                obj = copy(s);
            end
        end
    end
    
    %================================================================
    methods (Static, Access = {?nav.algs.internal.MapUtils, ...
                            ?matlabshared.autonomous.map.internal.InternalAccess})
        function logodds = probToLogodds(prob)
        %probToLogodds Get log-odds value from probabilities
            logodds = log(prob./(1-prob));
        end

        function probability = logoddsToProb(logodds)
        %logoddsToProb Get probabilities from log-odds values
            probability = 1 - 1./(1 + exp(logodds));
        end
    end

    methods (Access = {?occupancyMap, ?nav.algs.internal.InternalAccess, ...
                       ?matlabshared.autonomous.map.internal.InternalAccess})
        function occupied = checkOccupancyImpl(obj, locs, matSize, isGrid, isLocal)
            if isempty(matSize)
                % Individual points
                if isGrid
                    value = obj.getValueAtIndicesInternal(locs,-1);
                elseif isLocal
                    value = obj.getValueLocalAtIndicesImpl(locs,-1);
                else
                    value = obj.getValueWorldAtIndicesImpl(locs,-1);
                end
            else
                % Block syntax
                if isGrid
                    value = obj.getValueGridBlockImpl(locs, matSize(1), matSize(2),-1);
                elseif isLocal
                    value = obj.getValueLocalBlockImpl(locs, matSize(1), matSize(2),-1);
                else
                    value = obj.getValueWorldBlockImpl(locs, matSize(1), matSize(2),-1);
                end
            end
            
            freeIdx = (value < obj.FreeThresholdIntLogodds);
            occIdx = (value > obj.OccupiedThresholdIntLogodds);
            
            occupied = repmat(-1,size(value,1),size(value,2));
            occupied(freeIdx) = 0;
            occupied(occIdx) = 1;
        end
                   
        function logodds = probToIntLogodds(obj, prob)
        %probToIntLogodds Convert probability to int16 log-odds

             % Find elements that fall outside saturation limits
            m1 = prob < obj.ProbabilitySaturation(1);
            m2 = prob > obj.ProbabilitySaturation(2);

            if coder.target('MATLAB')
                % Use logical indexing and cached interpolant during MATLAB
                % execution.
                prob(m1) = obj.ProbabilitySaturation(1);
                prob(m2) = obj.ProbabilitySaturation(2);
                logodds = int16(obj.ProbInterpolant(prob));
            else
                % Generate lookup table and don't use logical indexing to
                % avoid varsize signal requirement during ERT codegen.
                prob = prob.*~m1 + obj.ProbabilitySaturation(1).*m1;
                prob = prob.*~m2 + obj.ProbabilitySaturation(2).*m2;
                logodds = int16(interp1(obj.LookupTable, ...
                                    single(obj.LinSpaceInt16),prob,'nearest', 'extrap'));
            end


        end

        function probability = intLogoddsToProb(obj, logodds)
        %intLogoddsToProb Convert int16 log-odds to probability
            if coder.target('MATLAB')
                probability = reshape(nav.algs.internal.mex.intLogoddsToProb(...
                    obj.LookupTable, obj.LinSpaceInt16, logodds(:)),size(logodds));
            else
                probability = reshape(nav.algs.internal.impl.intLogoddsToProb(...
                    obj.LookupTable, obj.LinSpaceInt16, logodds(:)),size(logodds));
            end
            
        end

        function logodds = probToIntLogoddsMaxSat(obj, prob)
        %probToIntLogoddsMaxSat Convert probability to int16 log-odds
        %   This method uses maximum possible saturation. Required for
        %   update occupancy.

            % Find elements that fall outside saturation limits
            m1 = prob < obj.ProbMaxSaturation(1);
            m2 = prob > obj.ProbMaxSaturation(2);
 
            if coder.target('MATLAB')
                % Use logical indexing and cached interpolant during MATLAB
                % execution.
                prob(m1) = obj.ProbMaxSaturation(1);
                prob(m2) = obj.ProbMaxSaturation(2);
                logodds = int16(obj.ProbInterpolant(prob));
            else
                % Generate lookup table and don't use logical indexing to
                % avoid varsize signal requirement during ERT codegen.
                prob = prob.*~m1 + obj.ProbMaxSaturation(1).*m1;
                prob = prob.*~m2 + obj.ProbMaxSaturation(2).*m2;
                logodds = int16(interp1(obj.LookupTable, ...
                                    single(obj.LinSpaceInt16),prob,'nearest', 'extrap'));
            end
        end
        
        function maxSat = getMaxSaturation(obj,idxs)
        %getMaxSaturation Get discretized max saturation
            minMax = [intmin(obj.DefaultType) intmax(obj.DefaultType)];
            maxSat = obj.intLogoddsToProb(minMax(idxs));
        end
        
        function validInverseModel = validateInverseModel(obj, inverseModel)
        %validateInverseModel Validate inverse model logodds values

            validateattributes(inverseModel, {'numeric'}, ...
                               {'real', 'nonnan', 'finite', 'nonempty', 'numel', 2}, 'insertRay', 'invmodel');

            invModel = inverseModel(:)';

            if coder.target('MATLAB')
                firstElementMsg = message('nav:navalgs:occgrid:FirstElement').getString;
                secondElementMsg = message('nav:navalgs:occgrid:SecondElement').getString;
            else
                % Hard code error string for code generation as coder
                % does not support getting string from catalog.
                firstElementMsg = 'first element of';
                secondElementMsg = 'second element of';
            end

            validateattributes(invModel(1,1), {'numeric'}, ...
                               {'>=', 0, '<=', 0.5, 'scalar'}, 'insertRay', [firstElementMsg 'invModel']);
            validateattributes(invModel(1,2), {'numeric'}, ...
                               {'>=', 0.5, '<=', 1, 'scalar'}, 'insertRay', [secondElementMsg 'invModel']);

            validInverseModel = obj.probToIntLogoddsMaxSat(double(invModel));
        end
        
        function updateIndices(obj,isGrid,isLocal,locations,value)
        %updateIndices Update one or more cells with occupancy readings
            nav.algs.internal.MapUtils.validateOccupancyValues(value(:),size(locations,1),'updateOccupancy','VAL',2);
            % Individual points
            if isGrid
                internalValue = obj.getValueAtIndicesInternal(locations);
                fSet = @(indices,vals)obj.setValueAtIndicesInternal(indices, vals(:));
            elseif isLocal
                internalValue = obj.getValueLocalAtIndicesImpl(locations);
                fSet = @(localXY,vals)obj.setValueAtIndicesLocalImpl(localXY, vals(:));
            else
                internalValue = obj.getValueWorldAtIndicesImpl(locations);
                fSet = @(worldXY,vals)obj.setValueAtIndicesWorldImpl(worldXY, vals(:));
            end

            if islogical(value)
                if isscalar(value)
                    vals = repmat(value, size(locations,1),1);
                else
                    vals = value;
                end
                
                % Two-step update must be performed when specifying indices
                % because the same cell may appear as both hit and miss.
                updateValuesHit = internalValue(vals,:) + obj.LogoddsHit;
                updateValuesHit(updateValuesHit > obj.ProbSatIntLogodds(2)) = obj.ProbSatIntLogodds(2);
                fSet(locations(vals,:),updateValuesHit);

                updateValuesMiss = internalValue(~vals,:) + obj.LogoddsMiss;
                updateValuesMiss(updateValuesMiss < obj.ProbSatIntLogodds(1)) = obj.ProbSatIntLogodds(1);
                fSet(locations(~vals,:), updateValuesMiss);
            else
                updateValues = obj.probToIntLogoddsMaxSat(double(value));
                combinedValues = min(max(updateValues(:)+internalValue,obj.ProbSatIntLogodds(1)),obj.ProbSatIntLogodds(2));
                fSet(locations,combinedValues);
            end
        end
        
        function updateRegion(obj, isGrid, isLocal, varargin)
        %updateRegion Update entire matrix or subregion with occupancy observations
            if numel(varargin) == 1
            %updateOccupancy(FULLMATRIX)
                vals = varargin{1};
                internalValue = obj.getValueAllImpl;
                fSet = @(vals)obj.setValueMatrixImpl(vals);
            else
            %updateOccupancy(CORNERLOCATION,SUBMATRIX,___)
                % Block syntax
                vals = varargin{2};
                if isGrid
                    gridCornerTopLeft = varargin{1};
                else
                    % Pad cartesian corner to eliminate floating-point error
                    cartCorner = varargin{1}+1/(2*obj.Resolution);
                    
                    % Convert bottom-left corner to grid
                    if isLocal
                        gridCornerBotLeft = obj.local2gridImpl(cartCorner);
                    else
                        gridCornerBotLeft = obj.world2gridImpl(cartCorner);
                    end
                    
                    % Calculate top-left grid corner
                    gridCornerTopLeft = [gridCornerBotLeft(1)-size(vals,1)+1, gridCornerBotLeft(2)];
                end
                internalValue = obj.getValueGridBlockImpl(gridCornerTopLeft, size(vals,1), size(vals,2));
                fSet = @(vals)obj.setBlockInternal(gridCornerTopLeft, vals);
            end
            
            if islogical(vals)
                % Convert logical values to hit/miss values
                updateValues = repmat(obj.LogoddsHit,size(internalValue));
                updateValues(~logical(vals)) = obj.LogoddsMiss;
            else
                updateValues = obj.probToIntLogoddsMaxSat(double(vals));
            end
            
            % Update internal matrix with new observations and bound
            % results between min/max thresholds.
            combinedValues = min(max(updateValues+internalValue,obj.ProbSatIntLogodds(1)),obj.ProbSatIntLogodds(2));
            fSet(combinedValues);
        end
    end

    methods(Access = protected)
        function copyImpl(cpObj, obj)
        %copyImpl Copies properties that were not set during construction
        
            % Assign properties to new object
            cpObj.OccupiedThresholdIntLogodds = obj.OccupiedThresholdIntLogodds;
            cpObj.FreeThresholdIntLogodds = obj.FreeThresholdIntLogodds;
            cpObj.ProbSatIntLogodds = obj.ProbSatIntLogodds;
            cpObj.LogoddsHit = obj.LogoddsHit;
            cpObj.LogoddsMiss = obj.LogoddsMiss;
            cpObj.ImageTag = 'occupancyMap';
            
            if (cpObj.Resolution == obj.Resolution)
                copyImpl@matlabshared.autonomous.internal.MapLayer(cpObj, obj);
            end
        end
        
        function postConstructSet(obj, varargin)
        %postConstructSet Set additional properties after internal construction
        %
        %   This function is called after construction to finish assigning
        %   specific to the occupancyMap.
        
            numCtorIn = numel(varargin);
            % Assign properties with default values
            if numCtorIn > 1 && isequal(class(varargin{1}),'occupancyMap')
                % Copy properties that were not set during construction
                % Set default properties
                obj.FreeThresholdIntLogodds = varargin{1}.FreeThresholdIntLogodds;
                obj.OccupiedThresholdIntLogodds = varargin{1}.OccupiedThresholdIntLogodds;
                obj.LogoddsHit = varargin{1}.LogoddsHit;
                obj.LogoddsMiss = varargin{1}.LogoddsMiss;
                obj.ProbSatIntLogodds = varargin{1}.ProbSatIntLogodds;
            else
                % Set default properties
                obj.FreeThresholdIntLogodds = obj.probToIntLogoddsMaxSat(0.2);
                obj.OccupiedThresholdIntLogodds = obj.probToIntLogoddsMaxSat(0.65);
                obj.LogoddsHit = obj.probToIntLogoddsMaxSat(0.7);
                obj.LogoddsMiss = obj.probToIntLogoddsMaxSat(0.4);
                obj.ProbSatIntLogodds = [intmin(obj.DefaultType), intmax(obj.DefaultType)];
            end
            
            if numCtorIn >= 1 && (isnumeric(varargin{1}) || islogical(varargin{1}))
                if numCtorIn == 1
                    obj.setOccupancy(varargin{1});
                elseif numCtorIn > 2 && ~(isnumeric(varargin{2}) || islogical(varargin{2}))
                    obj.setOccupancy(varargin{1});
                elseif ~isscalar(varargin{1}) || ~coder.internal.isConst(size(varargin{1}))
                    obj.setOccupancy(varargin{1});
                end
            end
        end
    end
    
    methods (Access = ?matlabshared.autonomous.map.internal.InternalAccess)
        function val = getDefaultValueConversion(obj)
        %getDefaultValueConversion Conversion function for get.DefaultValue
        %    Allows occupancyMap to override the MapLayer's DefaultValue
        %    getter
            val = obj.intLogoddsToProb(obj.DefaultValueInternal);
        end
        
        function convertedValue = setDefaultValueConversion(obj, value)
        %setDefaultValueConversion Conversion function for set.DefaultValue
        %    Allows occupancyMap to override the MapLayer's DefaultValue
        %    setter
            validateattributes(value, {'numeric','logical'}, {'nonempty','scalar','>=',0},'occupancyMap','DefaultValue');
            convertedValue = obj.probToIntLogodds(value);
        end
        
        function dataType = getDataTypeConversion(obj)
        %getDataTypeConversion Conversion function for get.DataType
        %    Allows occupancyMap to override the MapLayer's DataType
        %    getter
            dataType = class(obj.DefaultValue);
        end
    end

    properties (Access = {?nav.algs.internal.InternalAccess, ...
                          ?nav.algs.internal.MapUtils, ...
                          ?nav.algs.internal.GridAccess})
        %Logodds Internal log-odds representation for probability
        Logodds
        
        %AxesTag Axes tag used for quick show updates
        AxesTag = 'occupancyMap';

        %ImageTag Image tag used for quick show update
        ImageTag = 'occupancyMap';
    end
    
    methods (Static, Hidden)
        function name = getDefaultLayerName()
        %getDefaultLayerName Returns the Compile-time constant default name for occupancyMap objects
            name = 'probabilityLayer';
        end
        
        function defaultValue = getDefaultDefaultValue()
        %getDefaultDefaultValue Returns the Compile-time constant DefaultValue for occupancyMap objects
            defaultValue = int16(0);
        end
        
        function [nvPairs, useGridSizeInit, rows, cols, sz, depth] = parseGridVsMatrix(className, parseFcn, varargin)
        %parseGridVsMatrix Calculates size of matrix based on inputs and NV-pairs
            [nvPairs, useGridSizeInit, rows, cols, sz, depth] = nav.algs.internal.MapUtils.parseGridVsMatrix(className, parseFcn, varargin{:});
        end
        
        function validationFcn = getValDefaultValueFcn(methodName)
        %getValDefaultValueFcn Validator for DefaultValue
            validationFcn = @(name,val)validateattributes(val,{'numeric','logical'},{'nonempty','scalar','>=',0,'<=',1},methodName,name);
        end
        
        function validators = getValidators(methodName)
        %getValidators Returns validators for associated function calls
            
            invalidNV = occupancyMap.getInvalidNVPairFcn(methodName);
            validators = struct(...
                'Resolution',         {{1, occupancyMap.getValResolutionFcn(methodName)}},...
                'DefaultValue',       {{0, invalidNV}},...
                'GridOriginInLocal',  {{1, occupancyMap.getValRefFrameValueFcn(methodName)}},...
                'LocalOriginInWorld', {{1, occupancyMap.getValRefFrameValueFcn(methodName)}},...
                'LayerName',          {{1, occupancyMap.getValLayerNameFcn(methodName)}},...
                'GetTransformFcn',    {{0, invalidNV}},...
                'SetTransformFcn',    {{0, invalidNV}}, ...
                'UseGPU',             {{0, invalidNV}});
        end
    end
    
    methods (Hidden, Access = protected)
        function group = getPropertyGroups(obj)
            group = matlab.mixin.util.PropertyGroup;
            group.Title = 'mapLayer Properties';

            group.PropertyList = struct(...
                'OccupiedThreshold',    obj.OccupiedThreshold, ...
                'FreeThreshold',        obj.FreeThreshold, ...
                'ProbabilitySaturation',obj.ProbabilitySaturation, ...
                'LayerName',            obj.LayerName, ...
                'DataType',             obj.DataType, ...
                'DefaultValue',         obj.DefaultValue, ...
                'GridLocationInWorld',  obj.GridLocationInWorld, ...
                'GridOriginInLocal',    obj.GridOriginInLocal, ...
                'LocalOriginInWorld',   obj.LocalOriginInWorld, ...
                'Resolution',           obj.Resolution, ...
                'GridSize',             obj.GridSize, ...
                'XLocalLimits',         obj.XLocalLimits, ...
                'YLocalLimits',         obj.YLocalLimits, ...
                'XWorldLimits',         obj.XWorldLimits, ...
                'YWorldLimits',         obj.YWorldLimits ...
             );
        end
    end
end


function linSpace = getIntLineSpace()
%getIntLineSpace Generate line space for integer log-odds
    linSpace = intmin(occupancyMap.DefaultType):intmax(occupancyMap.DefaultType);
end


function lookup = createLookupTable()
%createLookupTable Create lookup table for integer log-odds to probability
    numOfPoints = abs(double(intmin(occupancyMap.DefaultType)))+...
        double(intmax(occupancyMap.DefaultType))+1;
    logOddsLimits = occupancyMap.probToLogodds(occupancyMap.ProbMaxSaturation);
    lookup = single(occupancyMap.logoddsToProb(...
        linspace(logOddsLimits(1), logOddsLimits(2), numOfPoints)));
end

function interpolant = getGriddedInterpolant()
%getGriddedInterpolant Creates the interpolant object for probability to integer log-odds
    if coder.target('MATLAB')
        interpolant = griddedInterpolant(createLookupTable(), single(getIntLineSpace()),'nearest');
    else
        % Codegen uses 'interp1' function
        interpolant = [];
    end
end
