classdef validatorVehicleCostmap < nav.StateValidator & ...
        matlabshared.tracking.internal.CustomDisplay & ...
        matlabshared.planning.internal.EnforceScalarHandle
    %validatorVehicleCostmap State validator based on 2-D costmap
    %   validatorVehicleCostmap validates states and discretized motions
    %   based on the value in a 2-D costmap. An occupied map area
    %   is interpreted as an invalid state. A free map area is interpreted as
    %   valid.
    %
    %   VALIDATOR = validatorVehicleCostmap creates a vehicle costmap validator
    %   associated with an SE2 state space with default settings.
    %
    %   VALIDATOR = validatorVehicleCostmap(SPACE) creates a validator in the
    %   state space, SPACE, given as an object from a class derived from
    %   nav.StateSpace. The given state space can have any desired
    %   dimensionality. The XYIndices property is set to [1 2] by default,
    %   which assumes the first two variables are [x y]. Adjust based on how
    %   your state space variables define the x and y coordinates.
    %
    %   VALIDATOR = validatorVehicleCostmap(SPACE,Name,Value) provides additional
    %   options specified by one or more Name,Value pair arguments. You can 
    %   specify several name-value pair arguments in any order as 
    %   Name1,Value1,...,NameN,ValueN:
    %
    %       'Map'           - Sets the Map property. Must be a
    %                         vehicleCostmap object
    %
    %                         Default: vehicleCostmap(10,10)
    %
    %       'XYIndices'     - Sets the XYIndices property, which specifies 
    %                         the position of x and y states in provided
    %                         state vectors. The values in XYINDICES must be
    %                         positive integers, less-than or equal to the 
    %                         number of state variables in the state space.
    %
    %                         Default: [1 2]
    %
    %       'ThetaIndex'    - Sets the ThetaIndex property, which specifies 
    %                         the position of theta in provided state 
    %                         vectors. The value in THETAINDEX must be a
    %                         positive integer, less than or equal to the 
    %                         number of state variables in the state space.
    %                         If set to NaN, the validation only checks 
    %                         whether xy-locations in the inflated map are 
    %                         occupied.
    %
    %                         Default: NaN
    %
    %   validatorVehicleCostmap properties:
    %      StateSpace         - State space for validating states
    %      Map                - Map used for validating states
    %      ValidationDistance - Interval for checking state validity
    %      XYIndices          - State variable mapping for xy-coordinates
    %      ThetaIndex         - State variable mapping for theta in state vector
    %
    %   validatorVehicleCostmap methods:
    %      copy              - Create deep copy of object
    %      isStateValid      - Check if state is valid
    %      isMotionValid     - Check if path between states is valid
    %
    %   Example:
    %       % Create a map from a matrix and visualize it
    %       map = vehicleCostmap(eye(10));
    %       figure
    %       plot(map)
    %
    %       % Create a state validator for SE2 states
    %       validator = validatorVehicleCostmap;
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
    %   See also vehicleCostmap, stateSpaceSE2.

    %   Copyright 2019-2021 The MathWorks, Inc.
    
    %#codegen

    properties (Constant, Access = private)
        %XYIndicesDefault - Default for XYIndices property
        XYIndicesDefault = [1 2]

        %ThetaIndexDefault - Default for ThetaIndex property
        ThetaIndexDefault = nan
        
        %ValidationDistanceDefault - Default for ValidationDistance property
        ValidationDistanceDefault = Inf
    end

    properties
        %Map Map used for validating states
        %   Map must be a vehicleCostmap object.
        %
        %   Default: vehicleCostmap(10, 10)
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
        %   costmap.
        %
        %   Default: [1 2]
        XYIndices = validatorVehicleCostmap.XYIndicesDefault
        
        %ThetaIndex State variable mapping for theta in state vector
        %   ThetaIndex indicates which column of the state vector corresponds
        %   to the angle of rotation about the z-axis. If defined, the
        %   costmap considers the orientation and geometry of the
        %   ego vehicle when checking if a state is free.
        %
        %   Default: nan
        ThetaIndex = validatorVehicleCostmap.ThetaIndexDefault
    end
    
    properties (Access = {?nav.algs.internal.InternalAccess})
        
        %OccupancyMapInternal Map vehicleCostMap to OccupancyBinaryMap
        OccupancyMapInternal;
        
    end        

    properties (Access = private, Transient)
        
        %StateIndexMap Maps incoming states to R2 ([x,y]) or SE2 ([x,y,Oz]) representation
        StateIndexMap
        
    end
    
    methods
        function obj = validatorVehicleCostmap(varargin)
        %validatorVehicleCostmap Constructor for validatorVehicleCostmap object

            narginchk(0,7);
        
            [stateSpace, map, xyInd, thetaInd] = validatorVehicleCostmap.parseConstuctor(varargin{:});

            % The state space object is validated in the StateValidator base class
            obj@nav.StateValidator(stateSpace);
            obj.XYIndices = xyInd;
            obj.ThetaIndex = thetaInd;
            obj.Map = map;
            obj.ValidationDistance = validatorVehicleCostmap.ValidationDistanceDefault;
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
        %       map = vehicleCostmap(eye(10));
        %       figure
        %       plot(map)
        %
        %       % Create a state validator for SE2 states
        %       validator = validatorVehicleCostmap
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

            narginchk(2,2);
            
            %Validate state input size and find the whether states are with
            %in map boundaries or not.
            [xyState, isInBounds] = validateState(obj, state, 'isStateValid');
            
            % For states inside the map, validate if they are occupied or not
            if size(xyState,2) == 2
                isValid = isInBounds & obj.Map.checkFreeWorldPoints(xyState,false);
            else
                isValid = isInBounds & obj.Map.checkFreePoses(xyState,false);
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
        %       map = vehicleCostmap(eye(10));
        %       map.CollisionChecker.InflationRadius = 0.5;
        %       figure;
        %       plot(map)
        %
        %       % Create a state validator for SE2 states and set the map
        %       validator = validatorVehicleCostmap
        %       validator.Map = map;
        %
        %       % Set a validation distance at motion path should be checked
        %       validator.ValidationDistance = 0.01
        %
        %       % A motion in the lower left corner is valid
        %       isMotionValid(validator,[0 0 0],[4 1 0])
        %
        %       % A motion across the map is invalid. Plot the last
        %       % valid point and the path.
        %       [isValid, lastValidState] = isMotionValid(validator,[1 1 0],[8 6.5 0])
        %
        %       hold on
        %       pathPoints = interpolate(validator.StateSpace,[1 1 0],[8 6.5 0],0:0.01:1);
        %       plot(pathPoints(:,1),pathPoints(:,2),".b")
        %       plot(lastValidState(1),lastValidState(2),".r")
        %       plot(lastValidState(1),lastValidState(2),"or","MarkerSize",10,"LineWidth",3)
        %       hold off

            narginchk(3,3);

            state1 = nav.internal.validation.validateStateVector(state1, ...
                                                              obj.StateSpace.NumStateVariables, 'isMotionValid', 'state1');
            state2 = nav.internal.validation.validateStateVector(state2, ...
                                                              obj.StateSpace.NumStateVariables, 'isMotionValid', 'state2');

            % Verify that state1 is valid
            if ~obj.isStateValid(state1)
                isValid = false;
                lastValid = nan(1,obj.StateSpace.NumStateVariables);
                return
            end

            % Interpolate between state1 and state2 with ValidationDistance
            dist = obj.StateSpace.distance(state1, state2);
            interval = obj.ValidationDistance/dist;
            interpStates = obj.StateSpace.interpolate(state1, state2, [0:interval:1 1]);

            % Check all interpolated states for validity
            interpValid = obj.isStateValid(interpStates);

            % Find the first invalid index. Note that the minimum non-empty
            % value of firstInvalidIdx is 2, since we always check
            % state1 and state2 and state1 is already verified above.
            firstInvalidIdx = find(~interpValid, 1);

            if isempty(firstInvalidIdx)
                % The whole motion is valid
                isValid = true;
                lastValid = state2;
            else
                isValid = false;
                lastValid = interpStates(firstInvalidIdx-1,:);
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
        %      validator = validatorVehicleCostmap;
        %      validator.ValidationDistance = 4.5
        %
        %      % Make a deep copy
        %      validator2 = COPY(validator)
        %
        %      % Verify that property values are the same
        %      validator.ValidationDistance == validator2.ValidationDistance

        % Set the state space through the constructor
            copyObj = validatorVehicleCostmap(copy(obj.StateSpace),'XYIndices',obj.XYIndices,'ThetaIndex',obj.ThetaIndex,'Map',copy(obj.Map));
            copyObj.ValidationDistance = obj.ValidationDistance;
        end
    end

    methods
        function set.Map(obj, map)
            %set.Map Setter for Map property

            validateattributes(map, {'vehicleCostmap'}, ...
                {'nonempty', 'scalar'}, 'validatorVehicleCostmap', 'Map');
            
            % We don't make a deep copy, but simply assign the handle
            obj.Map = map;

            %Convert vehicleCostmap to binaryOccupancyMap
            obj.OccupancyMapInternal = binaryOccupancyMap(obj.Map.checkOccupied, 1/obj.Map.CellSize); %#ok<MCSUP>
        end

        function set.XYIndices(obj, xyInd)
        %set.XYIndices Setter for XYIndices property

            validationAttributes = {'nonempty', 'vector', 'numel', 2, 'integer', '>=', 1};
            validateattributes(xyInd, {'double'}, validationAttributes, ...
                               'validatorVehicleCostmap', 'XYIndices');

            % Always save it as a row vector
            obj.XYIndices = reshape(xyInd, 1, []);
            
            % Update internal state mapping
            obj.setStateIndexMap();
        end

        function set.ThetaIndex(obj, thetaInd)
        %set.XYIndices Setter for XYIndices property
            if isnan(thetaInd)
                validateattributes(thetaInd, {'double'}, {'scalar'}, ...
                                   'validatorVehicleCostmap', 'ThetaIndex');
            else
                validateattributes(thetaInd, {'double'}, {'scalar', 'integer', '>=', 1}, ...
                                   'validatorVehicleCostmap', 'ThetaIndex');
            end
            
            obj.ThetaIndex = thetaInd;
            % Update internal state mapping
            obj.setStateIndexMap();
        end
        
        function set.ValidationDistance(obj, dist)
        %set.ValidationDistance Setter for ValidationDistance property

            validateattributes(dist, {'double'}, {'nonempty', 'scalar', 'real', 'nonnan', 'positive'}, ...
                               'validatorVehicleCostmap', 'ValidationDistance'); 

            obj.ValidationDistance = dist;
        end
    end

    methods (Static, Access = protected)
        function [stateSpace, map, xyInd, thetaInd] = parseConstuctor(varargin)
        %parseConstuctor Parses the inputs to the constructor
            p = matlabshared.autonomous.core.internal.NameValueParser({'XYIndices', 'ThetaIndex', 'Map'},...
                {validatorVehicleCostmap.XYIndicesDefault, validatorVehicleCostmap.ThetaIndexDefault, vehicleCostmap(zeros(10))});
            if nargin == 0
                stateSpace = stateSpaceSE2;
                p.parse(varargin{:});
            else
                % If any inputs are provided, stateSpace must come first
                stateSpace = varargin{1};
                p.parse(varargin{2:end});
            end
            xyInd = p.parameterValue('XYIndices');
            thetaInd = p.parameterValue('ThetaIndex');
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
                "ThetaIndex", obj.ThetaIndex,...
                "ValidationDistance", obj.ValidationDistance);
            propgrp = matlab.mixin.util.PropertyGroup(propList);
        end
        
        function setStateIndexMap(obj)
        %setStateIndexMap Updates the internal state mapping when XYIndices or ThetaIndex are changed
            if isnan(obj.ThetaIndex)
                obj.StateIndexMap = obj.XYIndices;
            else
                obj.StateIndexMap = [obj.XYIndices obj.ThetaIndex];
            end
        end
    end
    
    methods (Access = private)
        function [outState, isInBounds] = validateState(obj, state, methodName)
            %validateState Check state size and returns R2/SE2 state
            %   and true if state is within map boundary otherwise
            %   false. isInBounds is a column vector of logical values.
            
            nav.internal.validation.validateStateMatrix(state, nan, obj.StateSpace.NumStateVariables, ...
                                                        methodName, 'state');

            if any(obj.XYIndices > obj.StateSpace.NumStateVariables)
                coder.internal.error("nav:navalgs:statevalidoccmap:XYIndicesInvalid", obj.StateSpace.NumStateVariables);
            end
            if obj.ThetaIndex > obj.StateSpace.NumStateVariables
                coder.internal.error("nav:navalgs:statevalidvehiclecostmap:ThetaIndexInvalid", obj.StateSpace.NumStateVariables);
            end
            
            % Extract xy or xyTheta values
            sIdx = obj.StateIndexMap;
            outState = state(:,sIdx);
            
            % See which states are inside of the map boundary. States
            % outside the map extents are invalid.
            if nargout == 2
                % Calculate valid region
                ssLowerLims = obj.StateSpace.StateBounds(sIdx,1)';
                ssUpperLims = obj.StateSpace.StateBounds(sIdx,2)';
                ssLowerLims(1:2) = max(ssLowerLims(1:2), obj.Map.MapExtent([1 3]));
                ssUpperLims(1:2) = min(ssUpperLims(1:2), obj.Map.MapExtent([2 4]));
                ssLowerLims = repmat(ssLowerLims, size(outState,1), 1);
                ssUpperLims = repmat(ssUpperLims, size(outState,1), 1);
                
                if numel(sIdx) == 3
                    % Wrap heading if validated
                    outState(:,3) = matlabshared.tracking.internal.wrapToPi(outState(:,3));
                end
                
                % Verify states are within limits.
                isInBounds = all(outState >= ssLowerLims & outState <= ssUpperLims,2);
            end
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
            %   STATE) also returns XY-coordiante of nearest obstacle. If a
            %   STATE have more than one obstacles at same distance, it 
            %   returns MINDIST and XYPTSOBSTACLE form left-top obstacle (
            %   same behavior as bwdist).
            %
            %   Example:
            %       % Create a map from a matrix and visualize it
            %       map = vehicleCostmap(eye(10));
            %       figure
            %       plot(map)
            %
            %       % Create a state validator for SE2 states
            %       validator = validatorVehicleCostmap
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
            [xyState, isInBounds] = validateState(obj, state, 'nearestObstacleDistance');
            
            % Determine states are inside the map.
            if (~all(isInBounds))
                coder.internal.error('nav:navalgs:statevalidvehiclecostmap:StatesOutsideMap');
            end
            
            % Compute minimum distance from obstacles to each given states
            [minDist, xyPtsObstacle] = nav.algs.internal.clearance(xyState, ...
                obj.OccupancyMapInternal);
        end
    end
end