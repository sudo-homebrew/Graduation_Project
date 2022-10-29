classdef validatorOccupancyMap < nav.StateValidator & ...
        matlabshared.tracking.internal.CustomDisplay & ...
        matlabshared.planning.internal.EnforceScalarHandle & ...
        matlabshared.autonomous.map.internal.InternalAccess
    %VALIDATOROCCUPANCYMAP State validator based on 2-D grid map
    %   validatorOccupancyMap validates states and discretized motions
    %   based on the occupancy value in the same location of a 2-D occupancy map.
    %   If the state location in the map is occupied or unknown, the state
    %   is considered invalid. Only free space is considered valid.
    %
    %   VALIDATOR = validatorOccupancyMap creates an occupancy map validator
    %   associated with an SE2 state space with default settings.
    %
    %   VALIDATOR = validatorOccupancyMap(SPACE) creates a validator in the
    %   state space, SPACE, given as an object from a class derived from
    %   nav.StateSpace. The given state space can have any desired
    %   dimensionality. The XYIndices property is set to [1 2] by default,
    %   which assumes the first two variables are [x y]. Adjust based on how
    %   your state space defines the x and y coordinates.
    %
    %   VALIDATOR = validatorOccupancyMap(SPACE,Name,Value) provides additional
    %   options specified by one or more Name,Value pair arguments. You can 
    %   specify several name-value pair arguments in any order as 
    %   Name1,Value1,...,NameN,ValueN:
    %
    %       'Map'           - Sets the Map property. Must be an occupancy map 
    %                         object, which can be of type binaryOccupancyMap
    %                         or occupancyMap.
    %
    %                         Default: binaryOccupancyMap
    %
    %                            NOTE: Map type cannot be changed in
    %                            generated code after the validator is constructed. 
    %                            This name-value pair must be provided when using
    %                            non-default map types in codegen workflows.
    %
    %       'XYIndices'     - Sets the XYIndices property, which specifies 
    %                         the position of x and y states in provided
    %                         state vectors. The values in XYINDICES must be
    %                         positive integers, less-than or equal to the 
    %                         number of state variables in the state space.
    %
    %                         Default: [1 2]
    %
    %   validatorOccupancyMap properties:
    %      StateSpace         - State space for validating states
    %      Map                - Map used for validating states
    %      ValidationDistance - Interval for checking state validity
    %      XYIndices          - State variable mapping for xy-coordinates
    %
    %   validatorOccupancyMap methods:
    %      copy              - Create deep copy of object
    %      isStateValid      - Check if state is valid
    %      isMotionValid     - Check if path between states is valid
    %
    %   Example:
    %       % Create a map from a matrix and visualize it
    %       map = occupancyMap(eye(10));
    %       figure
    %       show(map)
    %
    %       % Create a state validator for SE2 states
    %       validator = validatorOccupancyMap;
    %       validator.Map = map;
    %
    %       % State on the diagonal is invalid (occupied)
    %       isStateValid(validator,[4.5 5.5 0])
    %
    %       % State outside of map bounds is invalid
    %       isStateValid(validator,[11 11 pi])
    %
    %       % States at different locations are valid (unoccupied)
    %       isStateValid(validator,[2 1 0; 8.5 9.5 0])
    %
    %       % Set a non-infinite ValidationDistance
    %       validator.ValidationDistance = 0.1;
    %   
    %       % Validate a motion between two states
    %       isValid = isMotionValid(validator,[1 1 0],[1 3 0])
    %
    %   See also stateSpaceSE2.

    %   Copyright 2019-2021 The MathWorks, Inc.

    %#codegen
    
    properties (Constant, Access = private)
        %XYIndicesDefault - Default for XYIndices property
        XYIndicesDefault = [1 2]

        %ValidationDistanceDefault - Default for ValidationDistance property
        ValidationDistanceDefault = Inf
    end

    properties
        %Map Map used for validating states
        %   Map can either be a binaryOccupancyMap object or an
        %   occupancyMap object.
        %
        %   Default: binaryOccupancyMap(10, 10)
        Map

        %ValidationDistance Interval for checking state validity
        %   This property is used by the isMotionValid function to check
        %   state validity along the path. This distance needs to be positive.
        %
        %   Default: Inf (only check start state and end state)
        ValidationDistance

        %XYIndices State variable mapping for xy-coordinates
        %   XYIndices indicates what coordinates of the state vector correspond
        %   to the R^2 coordinates (x and y) that are used to access the
        %   occupancy map.
        %
        %   Default: [1 2]
        XYIndices
    end
    
    properties (Access = {?nav.algs.internal.InternalAccess})
        
        %SkipStateValidation Skip validation in certain member functions
        %   This switch is used by internal functions only
        SkipStateValidation
        
    end
    
    properties (Access = {?nav.algs.internal.InternalAccess})
        
        %ValidMatrix Temporary map binary matrix snapshot
        ValidMatrix
        
        %MapBounds Temporary map bounds snapshot
        MapBounds
    end

    methods
        function obj = validatorOccupancyMap(varargin)
        %VALIDATOROCCUPANCYMAP Constructor for validatorOccupancyMap object

            narginchk(0,5);

            [ss, map, xyInd] = validatorOccupancyMap.parseConstructor(varargin{:});
            
            % Construct the base class by default
            obj@nav.StateValidator(ss);
            
            % Set remaining properties
            obj.Map = map;
            obj.XYIndices = xyInd;
            obj.ValidationDistance = validatorOccupancyMap.ValidationDistanceDefault;
            
            % Validate inputs by default
            obj.SkipStateValidation = false;
            
            if ~coder.target('MATLAB')
                obj.ValidMatrix = coder.nullcopy(map.occupancyMatrix);
                obj.MapBounds = coder.nullcopy(zeros(2,2));
            end
        end
    end

    methods

        function isValid = isStateValid(obj, state)
        %isStateValid Check if state is valid
        %   ISVALID = isStateValid(OBJ, STATE) verifies if a set of given states,
        %   STATE, are valid. STATE must be an N-by-NumStateVariables
        %   matrix, representing N state samples within the StateSpace property.
        %   ISVALID is an n-element vector of logical values, with TRUE
        %   indicating that the corresponding state sample in STATE is
        %   valid.
        %
        %   Example:
        %       % Create a map from a matrix and visualize it
        %       map = binaryOccupancyMap(eye(10));
        %       figure
        %       show(map)
        %
        %       % Create a state validator for SE2 states
        %       validator = validatorOccupancyMap
        %       validator.Map = map;
        %
        %       % State on the diagonal is invalid (occupied)
        %       isStateValid(validator,[4.5 5.5 0])
        %
        %       % States at different locations are valid (unoccupied)
        %       isStateValid(validator,[2 1 0; 3.5 4.2 0])
        %
        %       % State that is outside of map bounds is invalid
        %       isStateValid(validator,[20 15 pi])

            narginchk(2,2);

            if ~obj.SkipStateValidation
                nav.internal.validation.validateStateMatrix(state, nan, obj.StateSpace.NumStateVariables, ...
                                                            'isStateValid', 'state');
            end
            
            xyInd = obj.XYIndices;
            if any(obj.XYIndices > obj.StateSpace.NumStateVariables)
                coder.internal.error('nav:navalgs:statevalidoccmap:XYIndicesInvalid', obj.StateSpace.NumStateVariables);
            end
            
            % Check states against StateSpace limits
            ssBounds = obj.StateSpace.StateBounds;
            ssLowerBounds = repmat(ssBounds(1:2,1)', size(state,1), 1);
            ssUpperBounds = repmat(ssBounds(1:2,2)', size(state,1), 1);
            
            isInBounds = all(state(:,xyInd) >= ssLowerBounds & state(:,xyInd) <= ssUpperBounds,2);
            
            % Check that state is inside bounds and not occupied
            if obj.SkipStateValidation
                isValid = isInBounds & obj.checkMapOccupancy(state(:,xyInd));
            else
                [isOccupied, idxInBounds] = obj.Map.checkOccupancy(state(:,xyInd));
                
                % Mark valid & inbound points
                isValid = isInBounds & idxInBounds & ~isOccupied;
            end
        end

        function [isValid, lastValid] = isMotionValid(obj, state1, state2)
        %isMotionValid Check if path between states is valid
        %   ISVALID = isMotionValid(OBJ, STATE1, STATE2) verifies if
        %   the path between STATE1 and STATE2 is valid. Both STATE1
        %   and STATE2 are vectors with length NumStateVariables.
        %   ISVALID is a logical scalar, with TRUE indicating that the
        %   given motion is valid. The start state STATE1 is assumed
        %   to be valid.
        %
        %   [ISVALID, LASTVALID] = isMotionValid(OBJ, STATE1, STATE2)
        %   also returns the last valid state, LASTVALID, if ISVALID
        %   is FALSE. LASTVALID is a state between STATE1 and STATE2. If
        %   STATE1 is invalid, LASTVALID returns a state-vector of NaNs.
        %
        %   Example:
        %       % Create a map from a matrix and visualize it
        %       map = binaryOccupancyMap(eye(10));
        %       figure;
        %       show(map)
        %
        %       % Create a state validator for SE2 states and set the map
        %       validator = validatorOccupancyMap
        %       validator.Map = map;
        %
        %       % Set a validation distance at motion path should be checked
        %       validator.ValidationDistance = 0.01
        %
        %       % A motion in the lower left corner is valid
        %       isMotionValid(validator,[0 0 0],[6 2 0])
        %
        %       % A motion across the map is invalid. Plot the last
        %       % valid point and the path.
        %       [isValid, lastValidState] = isMotionValid(validator,[2 2 0],[8 6.5 0])
        %
        %       hold on
        %       pathPoints = interpolate(validator.StateSpace,[2 2 0],[8 6.5 0],0:0.01:1);
        %       plot(pathPoints(:,1),pathPoints(:,2),".b")
        %       plot(lastValidState(1),lastValidState(2),".r")
        %       plot(lastValidState(1),lastValidState(2),"or","MarkerSize",10,"LineWidth",3)
        %       hold off

            if ~obj.SkipStateValidation
                narginchk(3,3);

                state1 = nav.internal.validation.validateStateVector(state1, ...
                                                                  obj.StateSpace.NumStateVariables, 'isMotionValid', 'state1');
                state2 = nav.internal.validation.validateStateVector(state2, ...
                                                                  obj.StateSpace.NumStateVariables, 'isMotionValid', 'state2');                
                
 
            end
            % Validate state1. This is required for plannerBiRRT.
            % Prior to 21a, the assumption of not validating state1 inside the
            % SkipStateValidation block was that state1 will always be a valid
            % state as it is part of the search tree. During BiRRT/plan,
            % however, on extending the goal tree, interpolation starts from the
            % random node to the goal tree - now, the random node (passed as
            % state1) might not be valid.  The minimum value of firstInvalidIdx
            % is assumed 2, but since the first state can be invalid the minimum
            % value can be 1. Now 1 - 1 is zero and that means lastValidState
            % will be indexing a 0 element in MATLAB which is an invalid index.
            if ~obj.isStateValid(state1)
                isValid = false;
                lastValid = nan(1,obj.StateSpace.NumStateVariables);
                return
            end
            % Interpolate between state1 and state2 with ValidationDistance
            dist = obj.StateSpace.distance(state1, state2);

            if coder.target("MATLAB")
                interval = obj.ValidationDistance/dist;
                interpStates = obj.StateSpace.interpolate(state1, state2, [0:interval:1 1]);
    
                % Check all interpolated states for validity
                isValid = obj.isStateValid(interpStates);

                if nargin == 1
                    if any(~isValid)
                        isValid = false;
                    else
                        isValid = true;
                    end
                else
                    % Find the first invalid index. Note that the maximum non-empty
                    % value of firstInvalidIdx can be 2, since we always check
                    % state1 and state2 and state1 is already verified above.
                    firstInvalidIdx = find(~isValid, 1);
    
                    if isempty(firstInvalidIdx)
                        % The whole motion is valid
                        isValid = true;
                        lastValid = state2;
                    else
                        isValid = false;
                        lastValid = interpStates(firstInvalidIdx-1,:);
                    end
                end
            else
                delta = obj.ValidationDistance;
                d = delta;
                n = 1;
                isValid = true;
                iState = state1;
                while d < dist
                    iState = obj.StateSpace.interpolate(state1,state2,d/dist);
                    if ~obj.isStateValid(iState)
                        isValid = false;
                        break;
                    end
                    n = n+1;
                    d = delta*n;
                end
                if isValid
                    isValid = obj.isStateValid(state2);
                end
                if nargout == 2
                    if isValid
                        lastValid = state2;
                    else
                        lastValid = iState;
                    end
                end
            end
        end

        function copyObj = copy(obj)
        %COPY Create deep copy of object
        %   COPYOBJ = COPY(OBJ) creates a deep copy of the state space
        %   object OBJ and return the new object in COPYOBJ. All
        %   data of OBJ is also present in COPYOBJ.
        %
        %   Example:
        %      % Create validator object and set custom validation distance
        %      validator = validatorOccupancyMap;
        %      validator.ValidationDistance = 4.5
        %
        %      % Make a deep copy
        %      validator2 = COPY(validator)
        %
        %      % Verify that property values are the same
        %      validator.ValidationDistance == validator2.ValidationDistance

        % Set the state space through the constructor
            copyObj = validatorOccupancyMap(copy(obj.StateSpace), "XYIndices", obj.XYIndices,...
                                                                  "Map", copy(obj.Map));
            copyObj.ValidationDistance = obj.ValidationDistance;
            copyObj.MapBounds = obj.MapBounds;
            copyObj.ValidMatrix = obj.ValidMatrix;
        end
    end

    methods
        function set.Map(obj, map)
            %set.Map Setter for Map property

            validateattributes(map, {'binaryOccupancyMap', 'occupancyMap'}, ...
                {'nonempty', 'scalar'}, 'validatorOccupancyMap', 'Map');

            % We don't make a deep copy, but simply assign the handle
            obj.Map = map;
        end

        function set.XYIndices(obj, xyInd)
        %set.XYIndices Setter for XYIndices property

            validationAttributes = {'nonempty', 'vector', 'numel', 2, 'integer', '>=', 1};
            validateattributes(xyInd, {'double'}, validationAttributes, ...
                               'validatorOccupancyMap', 'XYIndices');

            % Always save it as a row vector
            obj.XYIndices = reshape(xyInd, 1, []);
        end

        function set.ValidationDistance(obj, dist)
        %set.ValidationDistance Setter for ValidationDistance property

            validateattributes(dist, {'double'}, {'nonempty', 'scalar', 'real', 'nonnan', 'positive'}, ...
                               'validatorOccupancyMap', 'ValidationDistance'); 

            obj.ValidationDistance = dist;
        end
    end

    methods (Access = ?nav.algs.internal.InternalAccess)
        function configureValidatorForFastOccupancyCheck(obj)
            %configureValidatorForFastOccupancyCheck
            
            % Take a snapshot of the validator to reduce overhead while
            % planning
            obj.ValidMatrix = obj.Map.checkOccupancy;
            obj.MapBounds = [obj.Map.XWorldLimits; obj.Map.YWorldLimits];
        end
        
        function validPos = checkMapOccupancy(obj, stateXY)
            %checkMapOccupancy
            validPos = stateXY(:,1) >= obj.MapBounds(1) & stateXY(:,1) <= obj.MapBounds(3) & ...
                stateXY(:,2) >= obj.MapBounds(2) & stateXY(:,2) <= obj.MapBounds(4);
            
            % below is the same code used in occupancyMap.world2grid, but
            % "inlined" for performance purposes.
            gridInd = ceil([-obj.MapBounds(2) + stateXY(:,2),-obj.MapBounds(1) + stateXY(:,1)]*obj.Map.Resolution);
            
            % Lower-left map boundary is inclusive
            originIdx = abs(gridInd) < eps;
            if any(originIdx,'all')
                gridInd(originIdx) = 1;
            end

            % Convert grid-based location to matrix indices
            gridInd(:,1) = obj.Map.GridSize(1) + 1 - gridInd(:,1);
            
            linIdx = obj.Map.Buffer.Index.Size(1)*(gridInd(:,2)-1) + gridInd(:,1);
            
            if all(validPos)
                % All points are valid, check occupancy for entire set
                validPos = ~obj.ValidMatrix(linIdx);
            else
                % Only check points that are within map limits
                validPos(validPos) = ~obj.ValidMatrix(linIdx(validPos));
            end            
            
        end
    end
    
    methods (Static, Access = protected)
        function [stateSpace, map, xyInd] = parseConstructor(varargin)
        %parseConstructor Parses the inputs to the constructor
            p = matlabshared.autonomous.core.internal.NameValueParser({'XYIndices', 'Map'},...
                {validatorOccupancyMap.XYIndicesDefault, binaryOccupancyMap});
            if nargin == 0
                stateSpace = stateSpaceSE2;
                p.parse(varargin{:});
            else
                % If any inputs are provided, stateSpace must come first
                stateSpace = varargin{1};
                p.parse(varargin{2:end});
            end
            xyInd = p.parameterValue('XYIndices');
            map = p.parameterValue('Map');
        end
    end
    
    methods (Access = protected)
        function propgrp = getPropertyGroups(obj)
        %getPropertyGroups Custom property group display
        %   This function overrides the function in the
        %   CustomDisplay base class.
            propList = struct(...
                "Map", obj.Map,...
                "StateSpace", obj.StateSpace,...
                "XYIndices", obj.XYIndices,...
                "ValidationDistance", obj.ValidationDistance);
            propgrp = matlab.mixin.util.PropertyGroup(propList);
        end
    end
    
    methods (Access = {?nav.algs.internal.InternalAccess})
        
        function [minDist, xyPtsObstacle] = nearestObstacleDistance(obj, state)
            %nearestObstacleDistance Computes nearest obstacle distance
            %   MINDIST = nearestObstacleDistance(OBJ, STATE) computes
            %   distance from nearest obstacle grid cell center to given
            %   grid cell centers, enclosing STATE. STATE must be an
            %   N-by-NumStateVariables matrix, representing N state samples
            %   within the StateSpace property. MINDIST is an n-element
            %   vector of double values, with INF indicating that the
            %   environement is obstacle-free and 0 indicating that the
            %   state is on an obstacle.
            %
            %   [MINDIST, XYPTSOBSTACLE] = nearestObstacleDistance(OBJ,
            %   STATE) also returns XY-coordiante of nearest obstacle grid
            %   cell center. If a STATE have more than one obstacles at
            %   same distance, it returns MINDIST and XYPTSOBSTACLE form
            %   left-top obstacle ( same behavior as bwdist).
            %
            %   Example:
            %       % Create a map from a matrix and visualize it
            %       map = binaryOccupancyMap(eye(10));
            %       show(map)
            %
            %       % Create a state validator for SE2 states
            %       validator = validatorOccupancyMap
            %       validator.Map = map;
            %
            %       % State on the obstacle (occupied)
            %       nearestObstacleDistance(validator,[4.5 5.5 0])
            %
            %       % States at different locations (unoccupied)
            %       [minDist, xyPtsObstacle] = nearestObstacleDistance(validator,[2 1 0; 8.5 9.5 0])
            
            narginchk(2,2);
            
            %Validate state input size and find the whether states are with
            %in map boundaries or not.
            nav.internal.validation.validateStateMatrix(state, nan, obj.StateSpace.NumStateVariables, ...
                'nearestObstacleDistance', 'state');
            
            xyInd = obj.XYIndices;
            
            % Extract xy-coordinates
            xyState = state(:,xyInd);
            
            % See which states are inside of the map boundary. States
            % outside the map extents are invalid.
            % We need to check the boundaries, since
            % vehicleCostmap/checkFree errors if coordinates are outside
            % the map.
            isInBounds = xyState(:,1) >= obj.Map.XWorldLimits(1) & ...
                xyState(:,1) <= obj.Map.XWorldLimits(2) & ...
                xyState(:,2) >= obj.Map.YWorldLimits(1) & ...
                xyState(:,2) <= obj.Map.YWorldLimits(2);
            
            % Determine states are inside the map.
            if (~all(isInBounds))
                coder.internal.error('nav:navalgs:statevalidvehiclecostmap:StatesOutsideMap');
            end
            
            % Compute minimum distance from obstacles to each given states
            [minDist, xyPtsObstacle] = nav.algs.internal.clearance(xyState, obj.Map);
        end
    end
end
