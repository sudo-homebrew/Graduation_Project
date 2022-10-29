classdef validatorOccupancyMap3D < nav.StateValidator & ...
        matlabshared.tracking.internal.CustomDisplay & ...
        matlabshared.planning.internal.EnforceScalarHandle & ...
        matlabshared.autonomous.map.internal.InternalAccess
    %VALIDATOROCCUPANCYMAP3D State validator based on 3-D grid map
    %   The validatorOccupancyMap3D object validates states and discretized
    %   motions based on the occupancy value in a 3-D occupancy map. 
    %   Obstacle-free map locations are considered as valid states. Occupied
    %   and unknown map locations are interpreted as invalid states.  
    %
    %   VALIDATOR = validatorOccupancyMap3D creates a 3-D occupancy map
    %   validator associated with an SE3 state space with default settings.
    %
    %   VALIDATOR = validatorOccupancyMap3D(SPACE) creates a validator in 
    %   the given state space definition, SPACE, derived from nav.StateSpace.
    %   Unspecified properties have default values.
    %
    %   VALIDATOR = validatorOccupancyMap3D(SPACE,Name,Value) sets properties
    %   of the validator using one or more name-value pairs. Unspecified 
    %   properties have default values. Enclose each property name inside 
    %   single quotes (' ').
    %
    %   validatorOccupancyMap3D properties:
    %      StateSpace         - State space for validating states
    %      Map                - Map used for validating states
    %      ValidationDistance - Interval for checking state validity
    %      XYZIndices         - State variable mapping for xyz-coordinates
    %
    %   validatorOccupancyMap3D methods:
    %      copy              - Create deep copy of object
    %      isStateValid      - Check if state is valid
    %      isMotionValid     - Check if path between states is valid
    %
    %   Example:
    %       % Create a map from a matrix and visualize it
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
    %       % Create a state validator for SE3 states
    %       validator = validatorOccupancyMap3D
    %       validator.Map = omap;
    %
    %       % State is invalid (occupied)
    %       isStateValid(validator, [1 1 1 -0.1214 0.0760 -0.9529 -0.2672])
    %
    %   See also stateSpaceSE3, validatorOccupancyMap.

    %   Copyright 2020 The MathWorks, Inc.
    
    %#codegen

    properties (Constant, Access = private)
        %XYZIndicesDefault - Default for XYZIndices property
        XYZIndicesDefault = [1 2 3]
        
        %ValidationDistanceDefault - Default for ValidationDistance property
        ValidationDistanceDefault = Inf
    end

    properties
        %Map Map used for validating states
        %   Map used for validating states, specified as an occupancyMap3D 
        %   object.
        %
        %   Default: occupancyMap3D
        Map

        %ValidationDistance Interval for checking state validity
        %   Interval for sampling between states and checking state validity,
        %   specified as a positive numeric scalar.
        %
        %   Default: Inf (only check start state and end state)
        ValidationDistance

        %XYZIndices State variable mapping for xyz-coordinates
        %   XYZIndices State variable mapping for xyz-coordinates in state 
        %   vector, specified as a three-element vector, [xIdx,yIdx,zIdx].
        %
        %   Default: [1 2 3]
        XYZIndices
    end
    
    properties (Access = {?nav.algs.internal.InternalAccess})
        
        %SkipStateValidation Skip validation in certain member functions
        %   This switch is used by internal functions only
        SkipStateValidation
        
    end
    
    methods
        function obj = validatorOccupancyMap3D(varargin)
            %VALIDATOROCCUPANCYMAP3D Constructor for validatorOccupancyMap3D object
            
            narginchk(0,7);
            
            [ss, map, xyzInd, vaidationDist] = validatorOccupancyMap3D.parseConstuctor(varargin{:});
            
            % Construct the base class by default
            obj@nav.StateValidator(ss);
            
            % Set remaining properties
            obj.Map = map;
            obj.XYZIndices = xyzInd;
            obj.ValidationDistance = vaidationDist;
            
            % Validate inputs by default
            obj.SkipStateValidation = false;
        end 
        
        function isValid = isStateValid(obj,state)
            %isStateValid Check if state is valid
            %   ISVALID = isStateValid(OBJ, STATE) verifies if a set of given
            %   states, STATE, are valid. Specify states as an m-by-n matrix. 
            %   n is the dimension of the state space specified in 
            %   StateSpace property. m is the number of states to validate. 
            %   ISVALID is an m-element vector of logical values, with TRUE
            %   indicating that the corresponding state sample in STATE is
            %   valid.
            %
            %   Example:
            %       % Create a map from a matrix and visualize it
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
            %       % Create a state validator for SE3 states
            %       validator = validatorOccupancyMap3D
            %       validator.Map = omap;
            %
            %       % State is invalid (occupied)
            %       isStateValid(validator, [1 1 1 -0.1214 0.0760 -0.9529 -0.2672])
            
            narginchk(2,2);
            
            if ~obj.SkipStateValidation
                nav.internal.validation.validateStateMatrix(state, nan, obj.StateSpace.NumStateVariables, ...
                    'isStateValid', 'state');
            end
            
            xyzInd = obj.XYZIndices;
            if any(obj.XYZIndices > obj.StateSpace.NumStateVariables)
                coder.internal.error('nav:navalgs:statevalidoccmap3D:XYZIndicesInvalid', obj.StateSpace.NumStateVariables);
            end
            
            isOccupied = obj.Map.checkOccupancy(state(:,xyzInd));
            isValid = ~isOccupied;
        end
        
        function [isValid, lastValid] = isMotionValid(obj, state1, state2)
            %isMotionValid Check if path between states is valid
            %   ISVALID = isMotionValid(VALIDATOROBJ, STATE1, STATE2) 
            %   verifies if the motion between two states is valid by 
            %   interpolating between the states. Specify states as an  
            %   n-element vector. n is the dimension of the state space  
            %   specified inStateSpace property. ISVALID is a logical scalar, 
            %   with TRUE indicating that the given motion is valid. The 
            %   start state STATE1 is assumed to be valid.  
            %
            %   [ISVALID, LASTVALID] = isMotionValid(VALIDATOROBJ, STATE1, STATE2) 
            %   returns the last valid state, LASTVALID, as the second
            %   output.
            %
            %   Example:
            %       % Create a map from a matrix and visualize it
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
            %       % Inflate a map.
            %       robotRadius = 0.2;
            %       safetyRadius = 0.3;
            %       inflationRadius = robotRadius + safetyRadius;
            %       inflate(omap, inflationRadius);
            %
            %       validState = [7.0605    8.2346    4.3874];
            %       setOccupancy(omap, validState, 0);
            %
            %       inValidState = [0.3183    6.9483    3.8156];
            %       setOccupancy(omap, inValidState, 1);
            %
            %       % Visualize a 3D map of the environment.
            %       show(omap);
            %
            %       % Create a state validator for SE3 states
            %       validator = validatorOccupancyMap3D
            %       validator.Map = omap;
            %
            %       % Motion is invalid (occupied)
            %       isMotionValid(validator, [validState, -0.1214 0.0760 -0.9529 -0.2672], [inValidState -0.3856 0.6070 -0.3539 0.5980])
            
            if ~obj.SkipStateValidation
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
            end
            
            % Interpolate between state1 and state2 with ValidationDistance
            dist = obj.StateSpace.distance(state1, state2);
            interval = obj.ValidationDistance/dist;
            
            % Make sure state2 is always checked
            interpStates = obj.StateSpace.interpolate(state1, state2, [0:interval:1 1]);

            % Check all interpolated states for validity
            interpValid = obj.isStateValid(interpStates);            

            % Find the first invalid index. Note that the maximum non-empty
            % value of firstInvalidIdx can be 2, since we always check
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
            %COPY Create deep copy of state validator object
            %   VALIDATOROBJ2 = copy(VALIDATOROBJ1) creates a deep copy of 
            %   the state validator object, VALIDATOROBJ2, from the state 
            %   validator object, VALIDATOROBJ1.
            %
            %   Example:
            %      % Create validator object and set custom validation distance
            %      validator = validatorOccupancyMap3D;
            %      validator.ValidationDistance = 4.5
            %
            %      % Make a deep copy
            %      validator2 = copy(validator)
            %
            %      % Verify that property values are the same
            %      validator.ValidationDistance == validator2.ValidationDistance
            
            % Set the state space through the constructor
            copyObj = validatorOccupancyMap3D(copy(obj.StateSpace), "XYZIndices", obj.XYZIndices,...
                                                                  "Map", copy(obj.Map));
            copyObj.ValidationDistance = obj.ValidationDistance;
        end
    end
    
    methods
        function set.Map(obj, map)
            %set.Map Setter for Map property
            
            validateattributes(map, {'occupancyMap3D'}, ...
                {'nonempty', 'scalar'}, 'validatorOccupancyMap3D', 'Map');
            
            % We don't make a deep copy, but simply assign the handle
            obj.Map = map;
        end
        
        function set.XYZIndices(obj, xyzInd)
            %set.XYZIndices Setter for XYZIndices property
            
            validationAttributes = {'nonempty', 'vector', 'numel', 3, 'finite', 'integer', '>=', 1};
            validateattributes(xyzInd, {'double'}, validationAttributes, ...
                'validatorOccupancyMap3D', 'XYZIndices');
            
            % Always save it as a row vector
            obj.XYZIndices = reshape(xyzInd, 1, []);
        end
        
        function set.ValidationDistance(obj, dist)
            %set.ValidationDistance Setter for ValidationDistance property
            
            validateattributes(dist, {'double'}, {'nonempty', 'scalar', 'real', 'nonnan', 'positive'}, ...
                'validatorOccupancyMap3D', 'ValidationDistance');
            
            obj.ValidationDistance = dist;
        end
    end
    
    methods (Static, Access = protected)
        function [stateSpace, map, xyzInd, validationDist] = parseConstuctor(varargin)
            %parseConstructor Parses the inputs to determine property values
            %for the validatorOccupancyMap3D object
            
            p = matlabshared.autonomous.core.internal.NameValueParser({'XYZIndices', 'Map', 'ValidationDistance'},...
                {validatorOccupancyMap3D.XYZIndicesDefault, occupancyMap3D, validatorOccupancyMap3D.ValidationDistanceDefault});
            if nargin == 0
                stateSpace = stateSpaceSE3;
                p.parse(varargin{:});
            else
                % If any inputs are provided, stateSpace must come first
                stateSpace = varargin{1};
                p.parse(varargin{2:end});
            end
            map = p.parameterValue('Map');
            xyzInd = p.parameterValue('XYZIndices');
            validationDist = p.parameterValue('ValidationDistance');
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
                "XYZIndices", obj.XYZIndices,...
                "ValidationDistance", obj.ValidationDistance);
            propgrp = matlab.mixin.util.PropertyGroup(propList);
        end
    end
end