classdef plannerHybridAStar < nav.algs.internal.InternalAccess
%plannerHybridAStar Create Hybrid A* path planner
%   Hybrid A* Path Planner generates a smooth path in a given 2-D space
%   for vehicles with nonholonomic constraints.
%
%   planner = plannerHybridAStar(validator) creates a path planner object
%   using the Hybrid A* algorithm. Specify the StateValidator input as a
%   validatorOccupancyMap or validatorVehicleCostMap object.
%
%   planner = plannerHybridAStar(validator, "PropertyName", Value, ...) provides
%   additional options specified by one or more Name,Value pair arguments.
%   Name must appear inside single quotes (''). You can specify several
%   name-value pair arguments in any order as Name1,Value1,...,NameN,ValueN
%
%   plannerHybridAStar properties:
%       StateValidator            - State validator for planning
%       MinTurningRadius          - Minimum turning radius of the vehicle
%       MotionPrimitiveLength     - Length of motion primitives to be generated
%       NumMotionPrimitives       - Number of motion primitives to be generated
%       ForwardCost               - Cost multiplier to travel in forward direction
%       ReverseCost               - Cost multiplier to travel in reverse direction
%       DirectionSwitchingCost    - Additive cost for switching the motion direction
%       AnalyticExpansionInterval - Interval for attempting analytic expansion based on number of nodes created
%       InterpolationDistance     - Distance between interpolated poses in output path
%
%   plannerHybridAStar methods:
%       plan                      - Find a path between start and goal points
%       show                      - Visualize the planned path
%       copy                      - Create deep copy of the planner object
%
%   Notes
%   -----
%   - The Hybrid A* planner checks for collisions in the map by
%   interpolating the motion primitives and analytic expansion based on
%   the ValidationDistance property of the stateValidator object. If the
%   ValidationDistance property is set to Inf, the object interpolates
%   based on the cell size of the map specified in the state validator.
%   Inflate the occupancy map before assigning to the planner to account
%   for the vehicle size.
%
%   Example:
%
%       % Create a binary occupancy grid
%       map = binaryOccupancyMap(zeros(50,50),1);
%
%       % Create a stateValidator object
%       validator = validatorOccupancyMap;
%
%       % Assigning map to stateValidator object
%       validator.Map = map;
%
%       % Assign stateValidator object to the plannerHybridAStar
%       planner = plannerHybridAStar(validator);
%
%       % Reduce the value of the AnalyticExpansionInterval property to
%       % expand from the current node to reach the goal quicker using
%       % Reeds-Shepp connection.
%       planner.AnalyticExpansionInterval = 3;
%
%       % Find path between two points
%       pathObj = plan(planner, [5 5 pi/2], [45 45 pi/4]);
%
%       % Visualize the output path
%       show(planner)
%
%       % Adding legend to plot
%       legend
%
%   See also validatorOccupancyMap, binaryOccupancyMap, occupancyMap
%
%   References:
%
%   [1] Dmitri Dolgov, Sebastian Thrun, Michael Montemerlo, and James Diebel.
%       Practical Search Techniques in Path Planning for Autonomous Driving.
%       American Association for Artificial Intelligence, 2008.
%   [2] Janko Petereit, Thomas Emter, Christian W. Frey, Thomas Kopfstedt,
%       and Andreas Beutel. "Application of Hybrid A* to an Autonomous Mobile
%       Robot for Path Planning in Unstructured Outdoor Environments".
%       ROBOTIK 2012: 7th German Conference on Robotics. 2012, pp. 1-6.

%   Copyright 2019-2021 The MathWorks, Inc.

%#codegen

    % Properties related to input map and information related to it
    properties (Access = private)

        %Map Common variable to store all kind of supported maps
        Map;

        %Dimensions To store the dimensions of the input map
        Dimensions;

        %CellSize Square side length of each cell in world units
        CellSize;

    end

    % Properties related to nodes and their expansion in continuous space
    properties (Access = private)

        %visitedCellsFront To keep check on the cells which are been
        %   traversed by the tree by forward motion
        visitedCellsFront;

        %visitedCellsBack To keep check on the cells which are been
        %   traversed by the tree by reverse motion
        visitedCellsBack;

        %Heuristic2DMat Matrix having the length of the path stored
        %   considering the holonomic nature of the vehicle in presence
        %   of the obstacles
        Heuristic2DMat;

        %Heuristic2DObj Variable to store object to update 2D heuristic
        Heuristic2DObj;

        %PathFound To store the state of the path completion
        PathFound;

        %NumPointsMotionPrimitive Number of points of motion primitives to
        %   be considered according to the step size
        NumPointsMotionPrimitive;

    end

    % Properties related to analytic expansion
    properties (Access = private)

        %ExpansionPoint Point from where path is being expanded analytically
        ExpansionPoint;

        %AnalyticPathLength Length of the analytically expanded path
        AnalyticPathLength;

        %AnalyticPathSegments Length of each segment of the path
        AnalyticPathSegments;

        %AnalyticPathTypes Type of the expanded path
        AnalyticPathTypes;

    end

    % Properties related to show function
    properties (Access = private)

        %StartPose Start pose provided by user
        StartPose

        %GoalPose Goal pose provided by user
        GoalPose

        %PrimitivesData To store data of circular motion primitives
        PrimitivesData

        %LinesData To store data of straight motion primitives
        LinesData

        %PathData To store data of final path
        PathData

    end

    % Internal variables for the properties of class
    properties (Access = private)

        InternalMinTurningRadius = 0;
        InternalMotionPrimitiveLength = 0;
        InternalNumMotionPrimitives = 5;
        InternalForwardCost = 1;
        InternalReverseCost = 3;
        InternalDirectionSwitchingCost = 0;
        InternalAnalyticExpansionInterval = 5;
        InternalInterpolationDistance = 1;

    end

    % Properties for storing RGB values of colors
    properties (Access = {?plannerHybridAStar, ?nav.algs.internal.InternalAccess})

        DarkRed = [0.85 0.325 0.098];
        Red = [1 0 0];
        LightGreen = [0.001 0.745 0.233];
        LightBlue = [0.301 0.745 0.933];
        DarkBlue = [0.15 0.25 0.8];
        LightGrey = [0.6 0.6 0.6];
        DarkGrey = [0.3 0.245 0.233];
        Magenta = [1 0 1];

    end

    properties

        %StateValidator - State validator for planning.
        %
        %   Type: validatorOccupancyMap or validatorVehicleCostmap
        StateValidator;

    end

    properties (Dependent)

        %MotionPrimitiveLength - Length of motion primitives to be generated.
        %   Increase the length for large maps or sparse environments.
        %   Decrease the length in dense environments.
        %
        %   Note: The minimum value of 'MotionPrimitiveLength' must be greater
        %   than sqrt(2) * map_CellSize, and the maximum value cannot exceed
        %   one-fourth the length of the circumference of a circle based on
        %   the 'MinTurningRadius'.
        %
        %   Type: double
        %   Default: ceil(sqrt(2) * map_CellSize)
        MotionPrimitiveLength;

        %MinTurningRadius - Minimum turning radius of the vehicle.
        %
        %   Note: The value of 'MinTurningRadius' is set such that the
        %   'MotionPrimitiveLength' cannot exceed one-fourth the length of
        %   the circumference of a circle based on it.
        %
        %   Type: double
        %   Default: (2 * motion_primitive_length) / pi
        MinTurningRadius;

        %NumMotionPrimitives - Number of motion primitives to be generated.
        %
        %   Type: integer
        %   Default: 5
        NumMotionPrimitives;

        %ForwardCost - Cost multiplier to travel in forward direction.
        %   Increase to penalize forward motion.
        %
        %   Type: double
        %   Default: 1
        ForwardCost;

        %ReverseCost - Cost multiplier to travel in reverse direction.
        %   Increase to penalize reverse motion.
        %
        %   Type: double
        %   Default: 3
        ReverseCost;

        %DirectionSwitchingCost - Additive cost for switching the motion
        %   direction.
        %   Increase to penalize the direction switching.
        %
        %   Type: double
        %   Default: 0
        DirectionSwitchingCost;

        %AnalyticExpansionInterval Interval for attempting analytic expansion from lowest cost node available.
        %   Interval for attempting analytic expansion from the lowest cost
        %   node available at that instance, specified as a positive integer.
        %   The Hybrid A* path planner expands the motion primitives from
        %   the nodes with the lowest cost available at that instance:
        %   -   The number of nodes to be expanded depends upon the number
        %       of primitives to be generated in both the direction and
        %       their validity, the cycle repeats until AnalyticExpansionInterval
        %       is reached.
        %   -   The planner then attempts an analytic expansion to reach
        %       the goal pose from the tree using a Reeds-Shepp model. If
        %       the attempt fails, the planner repeats the cycle.
        %   Improve the algorithm performance by reducing the interval to
        %   increase the number of checks for a Reeds-Shepp connection to
        %   the final goal.
        %
        %   Type: integer
        %   Default: 5
        AnalyticExpansionInterval;

        %InterpolationDistance Distance between interpolated poses in output path
        %
        %   Type: double
        %   Default: 1
        InterpolationDistance;

    end

    methods (Access = {?plannerHybridAStar})

        function validateStartGoal(obj, start, goal)
        %validateStartGoal Validating start and goal poses

            validity = obj.StateValidator.isStateValid(start);
            coder.internal.errorIf(~validity, 'nav:navalgs:hybridastar:StartError');

            validity = obj.StateValidator.isStateValid(goal);
            coder.internal.errorIf(~validity, 'nav:navalgs:hybridastar:GoalError');

            % Storing start and goal positions after validation
            obj.StartPose = start;
            obj.GoalPose = goal;

        end

        function validateMotionPrimitiveLength(obj, length)
        %validateMotionPrimitiveLength To check the validity of length
        %   of motion primitive. Checking validity here as other
        %   properties of class should not be used in set method

            % Motion primitive should not have difference of heading
            % greater than pi/2
            errorValue = (pi*obj.InternalMinTurningRadius)/2;
            validateattributes(length, {'single', 'double'}, ...
                               {'nonempty', 'scalar', 'finite', 'real', 'positive', ...
                                '>', sqrt(2) * obj.CellSize, '<=', errorValue}, 'plannerHybridAStar', ...
                               'MotionPrimitiveLength');

        end

        function validateMinimumTurningRadius(obj, radius)
        %validateMinimumTurningRadius Validating the length of
        %   minimum turning radius

            % Motion primitive should not have difference of heading
            % greater than pi/2
            errorValue = (2*obj.InternalMotionPrimitiveLength)/pi;
            validateattributes(radius, {'single', 'double'}, ...
                               {'nonempty', 'scalar', 'finite', 'real', 'positive', '>=', errorValue}, ...
                               'plannerHybridAStar', 'MinTurningRadius');

        end

        function [defaultNames, defaultValues] = defaultNamesAndValues(obj)
        %defaultNamesAndValues Returning the properties name and the default
        %   values of the properties

            % Properties of the class
            defaultNames = { 'MotionPrimitiveLength', ...
                             'MinTurningRadius', ...
                             'NumMotionPrimitives', ...
                             'ForwardCost', ...
                             'ReverseCost', ...
                             'DirectionSwitchingCost', ...
                             'AnalyticExpansionInterval', ...
                             'InterpolationDistance'};

            % Default value of motion primitive length
            % Setting this value due to the fact that motion primitive
            % should leave the grid cell where the parent node lies
            primitiveLength = ceil(sqrt(2) * obj.CellSize);
            obj.InternalMotionPrimitiveLength = primitiveLength;

            % Default value of minimum turning radius
            % Setting this value due to the reason that motion primitive
            % length cannot exceed one-fourth the length of the
            % circumference of a circle based on the minimum turning radius
            minTurnRad = (2 * primitiveLength) / pi;

            % Making default not to be less than 2
            if minTurnRad < 2

                minTurnRad = 2;

            end

            obj.InternalMinTurningRadius = minTurnRad;

            % Default values of the properties of the class
            defaultValues = { primitiveLength, ...
                              minTurnRad, ...
                              obj.InternalNumMotionPrimitives, ...
                              obj.InternalForwardCost, ...
                              obj.InternalReverseCost, ...
                              obj.InternalDirectionSwitchingCost, ...
                              obj.InternalAnalyticExpansionInterval, ...
                              obj.InternalInterpolationDistance};

        end

        function assigningValuesToProperties(obj, parser)

            % Properties on which HLUT calculations depends on
            obj.validateMinimumTurningRadius(parameterValue(parser, 'MinTurningRadius'));
            validateattributes(parameterValue(parser, 'ForwardCost'), {'single', 'double'}, ...
                               {'nonempty', 'scalar', 'finite', 'real', '>=', 1}, ...
                               'plannerHybridAStar', 'ForwardCost');
            validateattributes(parameterValue(parser, 'ReverseCost'), {'single', 'double'}, ...
                               {'nonempty', 'scalar', 'finite', 'real', '>=', 1}, ...
                               'plannerHybridAStar', 'ReverseCost');

            obj.InternalMinTurningRadius = parameterValue(parser, 'MinTurningRadius');
            obj.InternalForwardCost = parameterValue(parser, 'ForwardCost');
            obj.InternalReverseCost = parameterValue(parser, 'ReverseCost');

            % Properties for which setter needs to be called
            obj.MotionPrimitiveLength = parameterValue(parser, 'MotionPrimitiveLength');
            obj.NumMotionPrimitives = parameterValue(parser, 'NumMotionPrimitives');
            obj.DirectionSwitchingCost = parameterValue(parser, 'DirectionSwitchingCost');
            obj.AnalyticExpansionInterval = parameterValue(parser, 'AnalyticExpansionInterval');
            obj.InterpolationDistance = parameterValue(parser, 'InterpolationDistance');

        end

        function initializeShowVariables(obj)
        %initializeShowVariables Initializing the variables which are
        %   used in show method

            % For circular motion primitives, data will be stored in the
            % form of [startPose goalPose ICRData radius direction]
            obj.PrimitivesData = inf * ones(obj.Dimensions(1)*obj.Dimensions(2)*obj.InternalNumMotionPrimitives, 11);

            % For straight motion primitives, data will be stored in the
            % form of [startPose goalPose direction]
            obj.LinesData = inf * ones(obj.Dimensions(1)*obj.Dimensions(2)*obj.InternalNumMotionPrimitives, 7);

            % For final path, data will be stored in the form of
            %[startPose goalPose ICRData radius direction]
            %ICRData will be [nan nan nan] and radius will be inf for
            %straight motion primitives
            obj.PathData = inf * ones(obj.Dimensions(1)*obj.Dimensions(2)*obj.InternalNumMotionPrimitives, 11);

        end

        function resetShowVariables(obj)
        %resetShowVariables Resetting the flags used in show method

            obj.PathFound = false;
            obj.StartPose = [];
            obj.GoalPose = [];

        end

        function getMapData(obj)
        %getMapData Getting the resolution and dimensions from all types
        %   of supported maps

            % Storing all the supported map internally
            if isa(obj.StateValidator, 'validatorOccupancyMap')

                obj.Map = obj.StateValidator.Map;

                % Converting vehicleCostmap to occupancyMap
            else

                % Extract map resolution
                resolution = 1/obj.StateValidator.Map.CellSize;

                % Create occupancyMap
                obj.Map = occupancyMap(obj.StateValidator.Map.Costmap, resolution);

                % Set properties
                obj.Map.FreeThreshold = obj.StateValidator.Map.FreeThreshold;
                obj.Map.OccupiedThreshold = obj.StateValidator.Map.OccupiedThreshold;
                obj.Map.GridLocationInWorld = obj.StateValidator.Map.MapExtent([1 3]);

            end

            % Extracting data out of the map
            obj.CellSize = 1/obj.Map.Resolution;
            obj.Dimensions = obj.Map.GridSize;

        end

        function cost = get3DHeuristic(obj, start, goal)
        %get3DHeuristic To get the h3d+ heuristic value

            [~, pathLength, ~] = matlabshared.planning.internal.ReedsSheppBuiltins.autonomousReedsSheppSegments( ...
                start, goal, obj.InternalMinTurningRadius, obj.InternalForwardCost, obj.InternalReverseCost, ...
                'optimal', {});
            pathLength = sum(abs(pathLength));
            cost = squeeze(pathLength);

        end

        function cost = get2DHeuristic(obj, point)
        %get2DHeuristic To get the h2d- heuristic value

            % Converting point in matrix frame
            matPoint = obj.Map.world2grid(point(:,1:2));

            % Getting indices of the points
            indices = matPoint(:,1) + size(obj.Heuristic2DMat,1) * (matPoint(:,2)-1);

            % Getting cost of the indices
            cost = obj.Heuristic2DMat(indices);
            goalPoint = obj.Map.world2grid(obj.GoalPose(1:2));

            % Cost will be infinite when the tree expansion goes to the
            % region which is not explored by A*. In that case, we need to
            % update this heuristic cost matrix
            while ~all(~isinf(cost))

                toFind = cost == inf;
                pointToFind = matPoint(toFind, 1:2);

                % Calling A* with updated start pose
                if isempty(obj.Heuristic2DObj.plan(goalPoint, pointToFind(1,:)))

                    return;

                end

                % Updating 2D heuristic after getting g-cost matrix
                obj.Heuristic2DMat = min(obj.Heuristic2DMat, obj.Heuristic2DObj.GCostMatrix(:,:,1));

                cost = obj.Heuristic2DMat(indices);

            end

        end

        function [result, finalPosesGridIndices] = isCircularPrimitiveValid(obj, initialPose, finalPoses, ICRsData, ...
                                                              radius, length, stepSize, direction)
        %isCircularPrimitiveValid To check whether a given motion
        %   primitives lies inside the map and is collision free

            % Making initial poses array to match the size of final poses
            initialPose = repmat(initialPose, size(finalPoses,1), 1);

            % Getting discrete poses for all motion primitives
            primitivePoses = plannerHybridAStar.getPosesCircularPrimitive( ...
                initialPose, finalPoses, ICRsData, radius, length, stepSize, 0);

            % Checking validity of the poses from all motion primitives
            validPrimitivePoses = obj.StateValidator.isStateValid(primitivePoses);

            % Reshaping the matrix according to the number of motion primitives
            validFinalPoses = all(reshape(validPrimitivePoses, obj.NumPointsMotionPrimitive-1, obj.InternalNumMotionPrimitives-1))';

            % Variable to return grid indices of final valid poses
            finalPosesGridIndices = [];

            if nnz(validFinalPoses)

                % Getting addresses of valid poses
                validFinalPosesIndex = validFinalPoses == 1;

                % Getting (x,y) coordinates of ends of valid poses
                validFinalPoints = finalPoses(validFinalPosesIndex, 1:2);

                % Getting indices for the grid cells where motion
                % primitives are ending up
                validFinalPointsGrid = obj.Map.world2grid(validFinalPoints);
                validFinalPointsGridIndices = (validFinalPointsGrid(:,2)-1) * obj.Dimensions(1) + validFinalPointsGrid(:,1);

                % Checking if motion primitives are not exploring
                % the cells which have been already explored
                if direction == 1

                    validFinalPoses(validFinalPosesIndex) = validFinalPoses(validFinalPosesIndex) ...
                        & ~obj.visitedCellsFront(validFinalPointsGridIndices);

                else

                    validFinalPoses(validFinalPosesIndex) = validFinalPoses(validFinalPosesIndex) ...
                        & ~obj.visitedCellsBack(validFinalPointsGridIndices);

                end

                finalPosesGridIndices = unique(validFinalPointsGridIndices);

            end

            % Getting the motion primitives which are valid at last
            result = validFinalPoses;

        end

        function result = checkAnalyticExpansion(obj, initialPose, finalPose, stepSize)
        %checkAnalyticExpansion To check if the analytically expanded curve is collision free

            % Storing the expansion point
            obj.ExpansionPoint = initialPose;

            % Getting the length of the expansion
            [~, obj.AnalyticPathSegments, obj.AnalyticPathTypes] = matlabshared.planning.internal.ReedsSheppBuiltins.autonomousReedsSheppSegments( ...
                initialPose(1:3), finalPose(1:3), obj.InternalMinTurningRadius, ...
                obj.InternalForwardCost, obj.InternalReverseCost, ...
                'optimal', {});
            obj.AnalyticPathLength = sum(abs(obj.AnalyticPathSegments));

            % Getting the interpolated points of the curve
            samples = linspace(stepSize, obj.AnalyticPathLength, obj.AnalyticPathLength/stepSize);
            segmentDirections = ones(numel(obj.AnalyticPathSegments), 1);
            segmentDirections(obj.AnalyticPathSegments < 0) = -1;
            expansionPoints = matlabshared.planning.internal.ReedsSheppBuiltins.autonomousReedsSheppInterpolateSegments( ...
                initialPose, finalPose, samples, obj.InternalMinTurningRadius, ...
                abs(obj.AnalyticPathSegments'), int32(segmentDirections'), ...
                uint32(obj.AnalyticPathTypes'));

            % If start and goal poses are same
            if isempty(expansionPoints)

                result = true;
                return;

            end

            % Validating the analytically expanded curve
            result = all(obj.StateValidator.isStateValid(expansionPoints));

        end

        function [fScore, gScore, hScore] = calculateCost(obj, newNodeData, currentNode, curvature, direction)
        %calculateCost To calculate f and g cost of node under operation

            gScore = obj.calculateGScore(currentNode(2), curvature, direction);

            % Considering the maximum of both the heuristic
            hScore = max(([obj.get2DHeuristic(newNodeData) obj.get3DHeuristic(newNodeData, obj.GoalPose)]), [], 2);

            % Checking if the direction of motion is being changed
            if currentNode(7) == 0 || currentNode(7) * direction == 1
                fScore = gScore + hScore;
            else
                fScore = gScore + hScore + obj.InternalDirectionSwitchingCost;
            end

        end

        function getFinalPathData(obj, pathData)
        %getFinalPathData Extracting data of motion primitives which
        %   are the part of final path

            % Index to fill final path array
            PathDataRow = 1;

            for i = 1:size(pathData, 1)-1

                % Motion primitive is circular
                if pathData(i,3) ~= pathData(i+1,3)

                    pathIndex = pathData(i, 1) == obj.PrimitivesData(:,1) & ...
                        pathData(i, 2) == obj.PrimitivesData(:, 2) & ...
                        pathData(i+1, 1) == obj.PrimitivesData(:,4) & ...
                        pathData(i+1, 2) == obj.PrimitivesData(:,5);

                    matchedPrimitiveData = obj.PrimitivesData(pathIndex, :);
                    obj.PathData(PathDataRow,:) = matchedPrimitiveData(1,:);

                    % Motion primitive is straight
                else

                    pathIndex = pathData(i, 1) == obj.LinesData(:,1) & ...
                        pathData(i, 2) == obj.LinesData(:,2) & ...
                        pathData(i+1, 1) == obj.LinesData(:,4) & ...
                        pathData(i+1, 2) == obj.LinesData(:,5);

                    % Filling up the data in the format as explained at declaration
                    obj.PathData(PathDataRow,:) = [obj.LinesData(pathIndex,1:6) ...
                                        nan nan nan inf obj.LinesData(pathIndex,7)];

                end

                PathDataRow = PathDataRow + 1;

            end

            obj.PathData(PathDataRow:end, :) = [];

        end

        function [path, dir] = getInterpolatedPath(obj)
        %getInterpolatedPath Generating the points and direction values
        %   according to the interpolation distance provided by the user

            % Getting the length of the part of the path which includes only motion
            % primitives( other than Reeds-Shepp path)
            primitivePathLength = obj.InternalMotionPrimitiveLength * size(obj.PathData,1);
            states = zeros(floor(primitivePathLength / obj.InternalInterpolationDistance), 3);
            directions = zeros(floor(primitivePathLength / obj.InternalInterpolationDistance), 1);

            % Iterating for the number of nodes lying on the path generated by
            % motion primitives
            for i = 1 : (primitivePathLength/obj.InternalInterpolationDistance)

                % Finding the index of the primitive
                primitiveIndex = fix((i * obj.InternalInterpolationDistance) / obj.InternalMotionPrimitiveLength) + 1;
                lengthOnPrimitive = rem((i*obj.InternalInterpolationDistance), obj.InternalMotionPrimitiveLength);

                % Terminating condition
                if i * obj.InternalInterpolationDistance >= primitivePathLength

                    states(end,:) = [];
                    directions(end) = [];
                    break;

                end

                % If motion primitive is straight
                if isinf(obj.PathData(primitiveIndex, 10))

                    pathPoints = plannerHybridAStar.getStraightPrimitiveData( ...
                        lengthOnPrimitive, obj.PathData(primitiveIndex,1:3), obj.PathData(primitiveIndex,11));
                    states(i,:) = pathPoints;
                    directions(i) = obj.PathData(primitiveIndex,11);

                    % If motion primitive is circular
                else

                    [pathPoints, ~] = plannerHybridAStar.getCircularPrimitiveData( ...
                        lengthOnPrimitive, 1/obj.PathData(primitiveIndex,10), ...
                        obj.PathData(primitiveIndex,1:3), obj.PathData(primitiveIndex,11));
                    states(i,:) = pathPoints;
                    directions(i) = obj.PathData(primitiveIndex,11);

                end

            end

            % Generating poses for analytically expanded path
            samples = linspace(obj.InternalInterpolationDistance, obj.AnalyticPathLength, obj.AnalyticPathLength/obj.InternalInterpolationDistance);

            if size(samples,2) ~= 0

                % Calculating distance at which direction switching happens on
                % analytically expanded path and adding it to samples if
                % not present
                getSwitchingMotion = diff(sign(nonzeros(obj.AnalyticPathSegments)));
                switchingDistance = cumsum(abs(obj.AnalyticPathSegments));
                directionSwitchingDistance = switchingDistance(getSwitchingMotion == 2 | getSwitchingMotion == -2);
                samples = unique([samples, directionSwitchingDistance']);

            end

            segmentDirections = ones(numel(obj.AnalyticPathSegments), 1);
            segmentDirections(obj.AnalyticPathSegments < 0) = -1;
            [expansionPoints, expansionDirs] = matlabshared.planning.internal.ReedsSheppBuiltins.autonomousReedsSheppInterpolateSegments( ...
                obj.ExpansionPoint, obj.GoalPose, [0, samples], ...
                obj.InternalMinTurningRadius, abs(obj.AnalyticPathSegments'), ...
                int32(segmentDirections'), uint32(obj.AnalyticPathTypes'));

            % Merging the direction values and poses to be returned to the
            % navPath object
            if ~isequal(obj.StartPose, obj.ExpansionPoint)
                path = [obj.StartPose; states; expansionPoints];
                dir = [obj.PathData(1,11); directions; expansionDirs];
            else
                path = expansionPoints;
                dir = expansionDirs;
            end

        end

        function drawStraightMotionPrimitives(obj, axHandle, direction)
        %drawStraightMotionPrimitives To draw straight motion primitives

            % Getting data as per the direction of primitives
            motionPrimitivesPoints = obj.LinesData(obj.LinesData(:,7) == direction,[1 2 4 5]);

            % Setting up colors as per the direction
            if direction == 1

                plotColor = obj.LightBlue;
                scatterColor = obj.DarkBlue;

            else

                plotColor = obj.LightGrey;
                scatterColor = obj.DarkGrey;

            end

            plot(axHandle, [motionPrimitivesPoints(:,1)'; motionPrimitivesPoints(:,3)'], ...
                 [motionPrimitivesPoints(:,2)'; motionPrimitivesPoints(:,4)'], ...
                 'Color', plotColor, 'HandleVisibility', 'off');
            scatter(axHandle, motionPrimitivesPoints(:,3)', motionPrimitivesPoints(:,4)', 'Marker', '.', ...
                    'MarkerFaceColor', scatterColor, 'MarkerEdgeColor',scatterColor, 'HandleVisibility', 'off');

        end

        function drawCircularMotionPrimitives(obj, axHandle, direction, stepSize)
        %drawCircularMotionPrimitives To draw circular motion primitives

            numPointsPerPrimitive = floor(obj.InternalMotionPrimitiveLength/stepSize) + 2;

            % Getting data as per the direction of primitives
            circularPrimitivesData = obj.PrimitivesData((obj.PrimitivesData(:, 11) == direction),:);

            % Matrix to be used for plotting
            circularPrimitivesPoints = nan((numPointsPerPrimitive+1) * size(circularPrimitivesData, 1), 2);

            % Getting points for circular motion primitives according
            % to step size for visualization
            for i = 1:size(circularPrimitivesData, 1)

                poses = plannerHybridAStar.getPosesCircularPrimitive( ...
                    circularPrimitivesData(i, 1:3), circularPrimitivesData(i, 4:6), ...
                    circularPrimitivesData(i, 7:9), circularPrimitivesData(i, 10), ...
                    obj.InternalMotionPrimitiveLength, stepSize, 1);

                rowToFill = ((numPointsPerPrimitive+1) * (i-1)) + 1;
                circularPrimitivesPoints(rowToFill:rowToFill+numPointsPerPrimitive-1, :) = poses(:,1:2);

            end

            % Setting up colors and message as per the direction
            if direction == 1

                plotColor = obj.LightBlue;
                scatterColor = obj.DarkBlue;
                msgStr = message('nav:navalgs:hybridastar:LegendFwdPrimitives').getString;

            else

                plotColor = obj.LightGrey;
                scatterColor = obj.DarkGrey;
                msgStr = message('nav:navalgs:hybridastar:LegendRevPrimitives').getString;

            end

            plot(axHandle, circularPrimitivesPoints(:,1), circularPrimitivesPoints(:,2), ...
                 'Color', plotColor, 'DisplayName', msgStr);
            scatter(axHandle, circularPrimitivesData(:,4)', circularPrimitivesData(:,5)', ...
                    'Marker', '.', 'MarkerFaceColor', scatterColor, 'MarkerEdgeColor', scatterColor, ...
                    'HandleVisibility', 'off');

        end

        function cleanUp(obj)
        %cleanUp To clean up after plan

            if isa(obj.StateValidator, 'validatorOccupancyMap')

                obj.StateValidator.SkipStateValidation = false;

            end

        end

    end

    methods (Access = {?plannerHybridAStar, ?nav.algs.internal.InternalAccess})

        function gScore = calculateGScore(obj, parentGScore, curvature, direction)

            gScore = repmat(parentGScore, size(curvature, 1), 1);

            % Checking the direction of the motion primitive
            if direction == 1

                gScore = gScore + obj.InternalForwardCost*(obj.InternalMotionPrimitiveLength + abs(curvature));

            elseif direction == -1

                gScore = gScore + obj.InternalReverseCost*(obj.InternalMotionPrimitiveLength + abs(curvature));

            end

        end

    end

    methods (Static, Access = {?plannerHybridAStar, ?nav.algs.internal.InternalAccess})

        function [newNodesPoses, ICRsData] = getCircularPrimitiveData(length, curvature, initialNodePose, direction)
        %getCircularPrimitiveData Calculating the poses of new
        %   nodes generated and ICR data of the circular motion
        %   primitives

            turningRadius = 1 ./ curvature;

            % Calculating the ICR(Instantaneous Center of Rotation) data
            turningAngle = length * curvature;
            centerX = initialNodePose(1) - turningRadius * sin(initialNodePose(3));
            centerY = initialNodePose(2) + turningRadius * cos(initialNodePose(3));

            % Calculating poses of the new nodes formed
            xNew = centerX + turningRadius .* sin(initialNodePose(3) + direction * turningAngle);
            yNew = centerY - turningRadius .* cos(initialNodePose(3) + direction * turningAngle);
            headingNew = initialNodePose(3) + direction * turningAngle;

            newNodesPoses = [xNew' yNew' headingNew'];
            ICRsData = [centerX' centerY' turningAngle'];

        end

        function newNodePose = getStraightPrimitiveData(length, initialNodePose, direction)
        %getStraightPrimitiveData Calculating the pose of new node generated

            % Calculating pose of the new node formed
            xNew = initialNodePose(1) + direction * length * cos(initialNodePose(3));
            yNew = initialNodePose(2) + direction * length * sin(initialNodePose(3));
            headingNew = initialNodePose(3);

            newNodePose = [xNew yNew headingNew];

        end

        function poses = getPosesCircularPrimitive(initialPose, finalPoses, ICRData, radius, length, stepSize, getInitialPose)
        %getPosesCircularPrimitive To generate discrete poses of circular 
        %   motion primitives based on given step size

            % Dividing the motion primitives as per the stepSize
            intervals = linspace(0, 1, length/stepSize + 2);
            expandedIntervals = ((finalPoses(:,3) - initialPose(:,3)) * intervals);
            angles = repmat(initialPose(:,3), 1, size(expandedIntervals,2)) + expandedIntervals;

            % Calculating final position of the primitives
            deltaX = sin(angles) .* repmat(radius', 1, size(angles,2));
            deltaY = cos(angles) .* repmat(radius', 1, size(angles,2));
            xPoints = repmat(ICRData(:,1), 1, size(deltaX,2)) + deltaX;
            yPoints = repmat(ICRData(:,2), 1, size(deltaY,2)) - deltaY;

            % For performance improvement, skipping the validity check for
            % initial pose as it must have been checked in earlier cycle
            if ~getInitialPose

                xPoints(:,1) = [];
                yPoints(:,1) = [];
                angles(:,1) = [];

            end

            xPoints = reshape(xPoints', size(xPoints,1)*size(xPoints,2), 1);
            yPoints = reshape(yPoints', size(yPoints,1)*size(yPoints,2), 1);
            angles = reshape(angles', size(angles,1)*size(angles,2), 1);

            poses = [xPoints yPoints angles];

        end

    end

    methods

        function obj = plannerHybridAStar(validator, varargin)
        %plannerHybridAStar Constructor
        % Default constructor taking stateValidator object as compulsory input

            % Checking the number of input arguments
            narginchk(1,17);

            obj.StateValidator = validator;

            % Getting default names and values
            [defaultNames, defaultValues] = defaultNamesAndValues(obj);

            % Parsing name-value pairs
            parser = robotics.core.internal.NameValueParser(defaultNames, defaultValues);
            parse(parser, varargin{:});

            % Assigning values to the class properties
            assigningValuesToProperties(obj, parser);

        end

        function [pathObj, dirVals, solnInfo] = plan(obj, start, goal)
        %plan Find obstacle-free path between two points
        %   pathObj = plan(planner, START, GOAL) computes an obstacle free
        %   path between start and goal points as [x y theta] vectors.
        %
        %   [pathObj, dirVals] = plan(planner, START, GOAL) also returns 
        %   the direction of motion for each pose along the path, dirVals,
        %   as a column vector. A value of 1 indicates forward direction 
        %   and a value of -1 indicates reverse direction. The function 
        %   returns an empty column vector when the planner is unable to
        %   find a path. 
        % 
        %   [pathObj, dirVals, solnInfo] = plan(planner, START, GOAL) also
        %   returns solnInfo that contains the solution information of the
        %   path planning as a structure.
        %
        %   solnInfo structure contains these fields: 
        %
        %         IsPathFound:      Boolean indicating whether a path is 
        %                           found. It returns as true(1) if a path
        %                           is found. Otherwise, it returns
        %                           false(0).
        %
        %            NumNodes:      Number of nodes in the search tree when
        %                           the planner terminates (excluding the 
        %                           root node).
        %
        %       NumIterations:      Number of planning iterations executed.
        %
        %            ExitFlag:      A number indicating why planner terminates
        %                           1 - Goal reached
        %                           2 - No feasible path exists between
        %                               start and goal
        %                           3 - No feasible path exists between start
        %                               and goal pose based upon planner's properties
        %
        %   Example:
        %
        %       % Create a binary occupancy grid
        %       map = binaryOccupancyMap(zeros(50,50),1);
        %
        %       % Create a stateValidator object
        %       validator = validatorOccupancyMap;
        %
        %       % Assigning map to stateValidator object
        %       validator.Map = map;
        %
        %       % Assign stateValidator object to the plannerHybridAStar
        %       planner = plannerHybridAStar(validator);
        %
        %       % Find path between two points
        %       pathObj = plan(planner, [5 5 pi/2], [45 45 pi/4]);
        %
        %       % Visualize the output path
        %       show(planner)
        %
        %   See also validatorOccupancyMap, binaryOccupancyMap, occupancyMap

            % Checking the number of input arguments
            narginchk(3,3);

            if coder.target('MATLAB')

                % To make sure that property is reset after plan method ended
                cleaner = onCleanup(@() obj.cleanUp);

            end

            % Validating the start and goal poses
            validateStartGoal(obj, start, goal);

            % Fetching map data
            getMapData(obj);

            % To reduce the calls for isStateValid method
            if isa(obj.StateValidator, 'validatorOccupancyMap')

                obj.StateValidator.SkipStateValidation = true;
                obj.StateValidator.configureValidatorForFastOccupancyCheck();

            end

            % Variable to check the path completion
            obj.PathFound = false;
            
            % Declaring path object to be returned
            pathObj = navPath(obj.StateValidator.StateSpace);
            
            % Assigning zero column vector to direction values
            dirVals = zeros(0, 1);

            if start == goal

                % Return the start pose in case start/goal poses are the same
                pathObj.append(start);
                
                % Return 1 in direction in case start/goal poses are the same
                dirVals = 1;
                
                solnInfo = struct('IsPathFound', true, 'NumNodes', 0, ...
                    'NumIterations', 0, 'ExitFlag', 1);
                
                return;

            end

            % Storing data of motion primitives in a particular format
            % which will be used later while plotting
            initializeShowVariables(obj);

            % Indexes to fill the primitives data
            primitivesDataRow = 1;
            linesDataRow = 1;

            % Variable to track the node expansion per cell
            obj.visitedCellsFront = false(obj.Dimensions);
            obj.visitedCellsBack = false(obj.Dimensions);

            % Creating object for getting 2D heuristic
            obj.Heuristic2DObj = plannerAStarGrid(obj.Map);

            % Declaring 2D heuristic table
            obj.Heuristic2DMat = inf * ones(obj.Dimensions);

            % The nodes in priority queue have the following format:
            % [fScore, gScore, hScore, x, y, theta, direction],
            % where fScore is the total score and the priority value.
            % gScore represents the cost from the initial pose, and
            % hScore is the heuristic cost to the goal pose. x, y, theta
            % are the pose of the current node. Direction is 1 for forward
            % motion and -1 for reverse motion.
            openSet = nav.algs.internal.PriorityQueue(7, 1);

            % The nodes in the node map have the following format:
            % [x, y, theta]
            nodeMap = nav.algs.internal.NodeMap(3);

            gScore = 0;
            hScore = max([obj.get2DHeuristic(obj.StartPose(1:2)) obj.get3DHeuristic(obj.StartPose, obj.GoalPose)]);
            
            % Condition when path from start to goal is not possible
            if hScore == inf
                if coder.target('MATLAB')
                    disp(message('nav:navalgs:hybridastar:NoPath').getString);
                end
                
                solnInfo = struct('IsPathFound', obj.PathFound, ...
                'NumNodes', 0, 'NumIterations', 0, 'ExitFlag', 2);
            
                return;
            end

            fScore = gScore + hScore;
            directionAtStart = 0;
            initNode = [fScore, gScore, hScore, obj.StartPose, directionAtStart];

            openSet.push(initNode);
            nodeMap.insertNode(obj.StartPose, 0);
            curvature = linspace(-1/obj.InternalMinTurningRadius, 1/obj.InternalMinTurningRadius, obj.InternalNumMotionPrimitives);
            curvature((obj.InternalNumMotionPrimitives + 1)/2) = [];

            % ValidationDistance property of StateValidator will be used as
            % step size. If ValidationDistance is set to inf, cell size of
            % map will be used as step size.
            if isinf(obj.StateValidator.ValidationDistance)
                stepSize = obj.CellSize;
                obj.StateValidator.ValidationDistance = obj.CellSize;
            else
                stepSize = obj.StateValidator.ValidationDistance;
            end

            obj.NumPointsMotionPrimitive = floor(obj.InternalMotionPrimitiveLength/stepSize) + 2;

            % Variables to store number of iterations and nodes
            numIterations = 0;
            numNodes = 0;
            
            % Main loop of Hybrid A*
            % Loop will be finished when Hybrid A* finds the path or when
            % there will be no space left to be explored by Hybrid A*
            while ~(openSet.isEmpty() || obj.PathFound)

                % Getting the current node and moving it from open list to
                % close list of Hybrid A*
                [currentNode, currentNodeId] = openSet.top();
                currentNodePose = currentNode(4:6);
                openSet.pop();
                numIterations = numIterations + 1;

                % Condition if path can be expanded analytically from the
                % node being pushed to closedSet as that is the node
                % available having the lowest cost
                if rem(numIterations, obj.InternalAnalyticExpansionInterval) == 0

                    % Checking if the analytic expansion from
                    % new node and goal is obstacle free
                    result = checkAnalyticExpansion(obj, currentNodePose, obj.GoalPose, stepSize);

                    if result

                        % Removing the rows with no data
                        obj.PrimitivesData(primitivesDataRow:end, :) = [];
                        obj.LinesData(linesDataRow:end, :) = [];

                        % Tracking the path from the node available with
                        % lowest cost to the starting node
                        % currentNode(1) is the node ID for the node available
                        % having lowest cost
                        pathPoses = nodeMap.traceBack(currentNodeId);
                        pathPoses = flipud(pathPoses);

                        %Extracting data of motion primitives which are the part of final path
                        getFinalPathData(obj, pathPoses);

                        % Generating the points and directions according to
                        % the interpolation distance provided by the user
                        [path, directions] = getInterpolatedPath(obj);
                        pathObj.append(path);
                        dirVals = directions;
                        
                        % Using continue in order to properly exit the code
                        % and generate proper response message
                        obj.PathFound = true;
                        continue;

                    end

                end

                % For forward and reverse motion primitives
                for direction = 1:-2:-1

                    % For circular motion primitives
                    [newNodesPoses, ICRsData] = plannerHybridAStar.getCircularPrimitiveData( ...
                        obj.InternalMotionPrimitiveLength, curvature, currentNodePose, direction);

                    % Checking validity of the circular motion primitives
                    [validPrimitives, newNodesPosesGridIndices] = isCircularPrimitiveValid(obj, currentNodePose, newNodesPoses, ICRsData, ...
                                                                      1./curvature, obj.InternalMotionPrimitiveLength, stepSize, direction);

                    % Removing data for invalid primitives
                    validCurvature = curvature';
                    invalidPrimitives = ~ validPrimitives;
                    newNodesPoses(invalidPrimitives, :) = [];
                    ICRsData(invalidPrimitives, :) = [];
                    validCurvature(invalidPrimitives, :) = [];
                    validPrimitives(invalidPrimitives) = [];
                    numValidPrimitives = nnz(validPrimitives);

                    if numValidPrimitives
                        % Store data when motion primitives are valid
                        obj.PrimitivesData(primitivesDataRow : primitivesDataRow + numValidPrimitives-1, :) = ...
                            [repmat(currentNodePose, numValidPrimitives, 1) newNodesPoses ICRsData ...
                             1./validCurvature repmat(direction, numValidPrimitives, 1)];
                        primitivesDataRow = primitivesDataRow + numValidPrimitives;

                        % Calculating cost of the nodes if they are valid
                        [fScore, gScore, hScore] = calculateCost(obj, newNodesPoses, currentNode, validCurvature, direction);
                        newNodes = [fScore, gScore, hScore, newNodesPoses, repmat(direction, numValidPrimitives, 1)];

                        for ii = 1:numValidPrimitives
                            openSet.push(newNodes(ii,:));
                            nodeMap.insertNode(newNodes(ii, 4:6), currentNodeId);
                        end
                        
                        % Adding valid nodes
                        numNodes = numNodes + numValidPrimitives;
                        
                    end

                    % For straight motion primitives
                    newNodePose = plannerHybridAStar.getStraightPrimitiveData( ...
                        obj.InternalMotionPrimitiveLength, currentNodePose, direction);

                    % Checking validity of the straight motion primitive
                    validPrimitive = obj.StateValidator.isMotionValid(currentNodePose, newNodePose);
                    newNodePoseGrid = obj.Map.world2grid(newNodePose(1:2));

                    % Checking if motion primitive is not exploring the
                    % cells which have been already explored
                    if direction * validPrimitive == 1

                        validPrimitive = validPrimitive & ~obj.visitedCellsFront(newNodePoseGrid(1), newNodePoseGrid(2));

                    elseif direction * validPrimitive == -1

                        validPrimitive = validPrimitive & ~obj.visitedCellsBack(newNodePoseGrid(1), newNodePoseGrid(2));

                    end

                    if validPrimitive
                        % Store data when motion primitive is valid
                        obj.LinesData(linesDataRow, :) = [currentNodePose newNodePose direction];
                        linesDataRow = linesDataRow + 1;

                        % Calculate cost of the node if it is valid
                        [fScore, gScore, hScore] = calculateCost(obj, newNodePose, currentNode, 0, direction);
                        newNodes = [fScore, gScore, hScore, newNodePose, direction];

                        openSet.push(newNodes);
                        nodeMap.insertNode(newNodes(:,4:6), currentNodeId);
                        
                        % Adding valid node
                        numNodes = numNodes + 1;

                    end

                    % Closing the cells from the map which are explored by
                    % circular primitives
                    if numValidPrimitives ~=0

                        if direction == 1

                            obj.visitedCellsFront(newNodesPosesGridIndices) = 1;

                        else

                            obj.visitedCellsBack(newNodesPosesGridIndices) = 1;
                        end

                    end

                    % Closing the cells from the map which are explored by
                    % straight primitives
                    if validPrimitive

                        if direction == 1

                            obj.visitedCellsFront(newNodePoseGrid(1), newNodePoseGrid(2)) = 1;

                        else

                            obj.visitedCellsBack(newNodePoseGrid(1), newNodePoseGrid(2)) = 1;

                        end

                    end

                end

            end

            if obj.PathFound
                
                solnInfo = struct('IsPathFound', obj.PathFound, ...
                    'NumNodes', numNodes, 'NumIterations', numIterations, 'ExitFlag', 1);
                
            else

                if coder.target('MATLAB')

                    disp(message('nav:navalgs:hybridastar:NoPath').getString);

                end
                
                solnInfo = struct('IsPathFound', obj.PathFound, ...
                    'NumNodes', numNodes, 'NumIterations', numIterations, 'ExitFlag', 3);
                
            end
          
        end

        function ax = show(obj, varargin)
        %show Visualize the planned path
        %   show(planner) plots the Hybrid A* expansion tree and the
        %   planned path in the map.
        %
        %   axHandle = show(planner) outputs the axes handle of the
        %   figure used to plot the path.
        %
        %   show(planner,"Name",Value) provided additional options
        %   specified by one or more name-value pairs. Options include:
        %
        %       'Parent'      - Axes handle for plotting the path.
        %
        %       'Tree'        - A Display the expansion tree, specified
        %                       as "on" or "off".
        %
        %                       Default: 'on'
        %
        %       'Path'        - Display the planned path, specified as
        %                       "on" or "off".
        %
        %                       Default: 'on'
        %
        %       'Positions'   - Display the start and goal points,
        %                       specified as "start", "goal", "both",
        %                       or "none".
        %
        %                       Default: 'Both'
        %
        %   Example:
        %       % Create a binary occupancy grid
        %       map = binaryOccupancyMap(zeros(50,50),1);
        %
        %       % Create a stateValidator object
        %       validator = validatorOccupancyMap;
        %
        %       % Assigning map to stateValidator object
        %       validator.Map = map;
        %
        %       % Assign stateValidator object to the plannerHybridAStar
        %       planner = plannerHybridAStar(validator);
        %
        %       % Find path between two points
        %       pathObj = plan(planner, [5 5 pi/2], [45 45 pi/4]);
        %
        %       % Visualize the output path
        %       show(planner)
        %
        %   See also validatorOccupancyMap, binaryOccupancyMap, occupancyMap

            % If called in code generation, throw incompatibility error
            coder.internal.errorIf(~coder.target('MATLAB'), 'nav:navalgs:hybridastar:NoCodegenSupportForShow', 'show');

            % Checking the number of input arguments
            narginchk(1,9);

            % Default names for show method
            defaultNames = { 'Parent', 'Tree', 'Positions', 'Path' };

            % Default values for show method
            defaultValues = { [], 'On', 'Both', 'On'};

            % Parsing name-value pairs
            showParser = robotics.core.internal.NameValueParser(defaultNames, defaultValues);
            parse(showParser, varargin{:});

            % Validating string type inputs and assigning input values to the properties
            drawTree = validatestring(parameterValue(showParser, 'Tree'), {'on', 'off'}, 'show', 'Tree');
            drawPositions = validatestring(parameterValue(showParser, 'Positions'), ...
                                           {'Start', 'Goal', 'Both', 'None'}, 'show', 'Positions');
            drawPath = validatestring(parameterValue(showParser, 'Path'), {'on', 'off'}, 'show', 'Path');

            % Assigning axis handle
            if isempty(parameterValue(showParser, 'Parent'))

                axHandle = newplot;

            else

                % Validating input axis handle
                robotics.internal.validation.validateAxesUIAxesHandle(parameterValue(showParser, 'Parent'));
                axHandle = parameterValue(showParser, 'Parent');

            end

            % Support for vehicleCostmap
            if isa(obj.StateValidator.Map, 'vehicleCostmap')

                plot(obj.StateValidator.Map, 'Parent', axHandle, 'Inflation', 'on');
                legend(axHandle, 'off');

                % For occupancyMap and binaryOccupancyMap
            else

                obj.StateValidator.Map.show('world', 'Parent', axHandle);

            end

            % Title of the plot
            title(axHandle, message('nav:navalgs:hybridastar:FigureTitle').getString);
            hold(axHandle, 'on');

            if obj.PathFound

                stepSize = 0.05;

                % Generating points for showing analytic path
                samples = linspace(0, obj.AnalyticPathLength, (obj.AnalyticPathLength/stepSize)+1);
                segmentDirections = ones(numel(obj.AnalyticPathSegments), 1);
                segmentDirections(obj.AnalyticPathSegments < 0) = -1;
                [expansionPoints, expansionDirections] = matlabshared.planning.internal.ReedsSheppBuiltins.autonomousReedsSheppInterpolateSegments( ...
                    obj.ExpansionPoint, obj.GoalPose, samples, ...
                    obj.InternalMinTurningRadius, abs(obj.AnalyticPathSegments'), ...
                    int32(segmentDirections'), uint32(obj.AnalyticPathTypes'));

                % Separating out the segments as per the directions
                expansionSegStart = expansionPoints(1:end-1,:);
                expansionSegEnd = expansionPoints(2:end,:);
                forwardExpansionPointsIndex = expansionDirections == 1;
                forwardExpansionSegIndex = forwardExpansionPointsIndex(1:end-1);
                forwardExpansionSeg = [expansionSegStart(forwardExpansionSegIndex,:) expansionSegEnd(forwardExpansionSegIndex,:)];
                reverseExpansionSeg = [expansionSegStart(~forwardExpansionSegIndex,:) expansionSegEnd(~forwardExpansionSegIndex,:)];

                % Arranging forward expansion path points separated by NaN
                forwardExpansionPathPoints = nan(3*size(forwardExpansionSeg,1), 2);
                forwardExpansionPathPoints(1:3:end-2,:) = [forwardExpansionSeg(:,1) forwardExpansionSeg(:,2)];
                forwardExpansionPathPoints(2:3:end-1,:) = [forwardExpansionSeg(:,4) forwardExpansionSeg(:,5)];

                % Arranging reverse expansion path points separated by NaN
                reverseExpansionPathPoints = nan(3*size(reverseExpansionSeg,1), 2);
                reverseExpansionPathPoints(1:3:end-2,:) = [reverseExpansionSeg(:,1) reverseExpansionSeg(:,2)];
                reverseExpansionPathPoints(2:3:end-1,:) = [reverseExpansionSeg(:,4) reverseExpansionSeg(:,5)];

                % Condition to display expansion tree
                if strcmpi(drawTree, 'On')

                    % For straight motion primitives in forward direction
                    obj.drawStraightMotionPrimitives(axHandle, 1);

                    % For straight motion primitives in reverse direction
                    obj.drawStraightMotionPrimitives(axHandle, -1);

                    % For circular motion primitives in forward direction
                    obj.drawCircularMotionPrimitives(axHandle, 1, stepSize);

                    % For circular motion primitives in reverse direction
                    obj.drawCircularMotionPrimitives(axHandle, -1, stepSize);

                    % Plotting analytically expanded path in forward
                    % direction
                    plot(axHandle, forwardExpansionPathPoints(:,1), forwardExpansionPathPoints(:,2), ...
                         'Color', obj.LightBlue, 'HandleVisibility', 'off');

                    % Plotting analytically expanded path in reverse
                    % direction
                    plot(axHandle, reverseExpansionPathPoints(:,1), reverseExpansionPathPoints(:,2), ...
                         'Color', obj.LightGrey, 'HandleVisibility', 'off');

                end

                % Condition for showing final path
                if strcmpi(drawPath, 'On')

                    % Separating circular and straight primitive data
                    finalStraightPathData = obj.PathData(isinf(obj.PathData(:,10)),:);
                    finalCircularPathData = obj.PathData(~isinf(obj.PathData(:,10)),:);

                    % Separating straight primitives as per the direction
                    forwardStraightPrimitives = finalStraightPathData(finalStraightPathData(:,11) == 1, 1:6);
                    reverseStraightPrimitives = finalStraightPathData(finalStraightPathData(:,11) == -1, 1:6);

                    % Arranging forward straight path points separated by NaN
                    forwardStraightPathPoints = nan(3*size(forwardStraightPrimitives,1), 2);
                    forwardStraightPathPoints(1:3:end-2,:) = [forwardStraightPrimitives(:,1) forwardStraightPrimitives(:,2)];
                    forwardStraightPathPoints(2:3:end-1,:) = [forwardStraightPrimitives(:,4) forwardStraightPrimitives(:,5)];

                    % Arranging reverse straight path points separated by NaN
                    reverseStraightPathPoints = nan(3*size(reverseStraightPrimitives,1), 2);
                    reverseStraightPathPoints(1:3:end-2,:) = [reverseStraightPrimitives(:,1) reverseStraightPrimitives(:,2)];
                    reverseStraightPathPoints(2:3:end-1,:) = [reverseStraightPrimitives(:,4) reverseStraightPrimitives(:,5)];

                    % Getting points for circular motion primitives according
                    % to step size for visualization
                    numPointsPerPrimitive = floor(obj.InternalMotionPrimitiveLength/stepSize) + 2;

                    % Separating circular primitives as per the direction
                    forwardCircularPrimitives = finalCircularPathData(finalCircularPathData(:,11) == 1,:);
                    reverseCircularPrimitives = finalCircularPathData(finalCircularPathData(:,11) == -1,:);

                    % Getting fine interpolated points in forward direction
                    forwardCircularPathPoints = nan((numPointsPerPrimitive+1) * size(forwardCircularPrimitives, 1), 2);
                    for i = 1:size(forwardCircularPrimitives,1)

                        poses = plannerHybridAStar.getPosesCircularPrimitive( ...
                            forwardCircularPrimitives(i, 1:3), forwardCircularPrimitives(i, 4:6), forwardCircularPrimitives(i, 7:9), ...
                            forwardCircularPrimitives(i, 10), obj.InternalMotionPrimitiveLength, stepSize, 1);
                        rowToFill = ((numPointsPerPrimitive+1) * (i-1)) + 1;
                        forwardCircularPathPoints(rowToFill:rowToFill+numPointsPerPrimitive-1, :) = poses(:,1:2);

                    end

                    % Getting fine interpolated points in reverse direction
                    reverseCircularPathPoints = nan((numPointsPerPrimitive+1) * size(reverseCircularPrimitives, 1), 2);
                    for i = 1:size(reverseCircularPrimitives,1)

                        poses = plannerHybridAStar.getPosesCircularPrimitive( ...
                            reverseCircularPrimitives(i, 1:3), reverseCircularPrimitives(i, 4:6), reverseCircularPrimitives(i, 7:9), ...
                            reverseCircularPrimitives(i, 10), obj.InternalMotionPrimitiveLength, stepSize, 1);
                        rowToFill = ((numPointsPerPrimitive+1) * (i-1)) + 1;
                        reverseCircularPathPoints(rowToFill:rowToFill+numPointsPerPrimitive-1, :) = poses(:,1:2);

                    end

                    % Combining all path points with same direction
                    forwardPath = [forwardStraightPathPoints; forwardCircularPathPoints; forwardExpansionPathPoints];
                    reversePath = [reverseStraightPathPoints; reverseCircularPathPoints; reverseExpansionPathPoints];

                    % Plotting path in forward direction
                    plot(axHandle, forwardPath(:,1), forwardPath(:,2), ...
                         'Color', obj.DarkRed, 'LineWidth', 3, 'DisplayName', message('nav:navalgs:hybridastar:LegendFwdPath').getString);

                    % Plotting path in reverse direction
                    plot(axHandle, reversePath(:,1), reversePath(:,2), ...
                         'Color', obj.Magenta, 'LineWidth', 3, 'DisplayName', message('nav:navalgs:hybridastar:LegendRevPath').getString);

                end

            end

            % Condition for showing start pose
            if (strcmpi(drawPositions, 'Start') || strcmpi(drawPositions, 'Both')) && ~isempty(obj.StartPose)

                scatter(axHandle, obj.StartPose(1), obj.StartPose(2), 'Marker', 'o', 'MarkerFaceColor', ...
                        obj.LightGreen, 'MarkerEdgeColor', obj.LightGreen, ...
                        'DisplayName', message('nav:navalgs:hybridastar:LegendStart').getString);

            end

            % Condition for showing goal pose
            if (strcmpi(drawPositions, 'Goal') || strcmpi(drawPositions, 'Both')) && ~isempty(obj.GoalPose)

                scatter(axHandle, obj.GoalPose(1), obj.GoalPose(2), 'Marker', 'o', 'MarkerFaceColor', ...
                        obj.Red, 'MarkerEdgeColor', obj.Red, 'DisplayName', message('nav:navalgs:hybridastar:LegendGoal').getString);

            end
            
            hold(axHandle, 'off');

            % Returning axis handle
            if nargout > 0

                ax = axHandle;

            end

        end

        function cpObj = copy(obj)
        %copy Create a deep copy of plannerHybridAStar object.
        %   plannerCopied = copy(planner) creates a deep copy of plannerHybridAStar
        %   object, planner, and returns the new object in plannerCopied.

            if isempty(obj) && coder.target('MATLAB')

                cpObj = plannerHybridAStar.empty;
                return;

            end

            % Construct a new object
            cpObj = plannerHybridAStar(obj.StateValidator);

            % Copy all public properties
            cpObj.MinTurningRadius = obj.MinTurningRadius;
            cpObj.MotionPrimitiveLength = obj.MotionPrimitiveLength;
            cpObj.NumMotionPrimitives = obj.NumMotionPrimitives;
            cpObj.ForwardCost = obj.ForwardCost;
            cpObj.ReverseCost = obj.ReverseCost;
            cpObj.DirectionSwitchingCost = obj.DirectionSwitchingCost;
            cpObj.AnalyticExpansionInterval = obj.AnalyticExpansionInterval;
            cpObj.InterpolationDistance = obj.InterpolationDistance;

        end

        function set.StateValidator(obj, validator)
        %set.StateValidator Setter for property StateValidator

            % Validate StateValidator input, all other types are currently not allowed
            validateattributes(validator, {'validatorOccupancyMap', 'validatorVehicleCostmap'}, {}, 'plannerHybridAStar', 'StateValidator');

            if isa(validator,'validatorVehicleCostmap')

                % Validate validatorVehicleCostmap input
                nav.internal.validation.validateValidatorVehicleCostmap(validator, "plannerHybridAStar", 'StateValidator');

            else

                % Validate validatorOccupancyMap input
                nav.internal.validation.validateValidatorOccupancyMap(validator, "plannerHybridAStar", 'StateValidator');

            end

            % Validate stateSpace property of StateValidator input
            coder.internal.errorIf(~ strcmp(validator.StateSpace.Name, 'SE2'), 'nav:navalgs:hybridastar:StateSpaceError');

            obj.StateValidator = validator;
            getMapData(obj);
            resetShowVariables(obj);

        end

        function set.MinTurningRadius(obj, radius)
        %set.MinTurningRadius Setter for property Minimum Turning Radius

            validateMinimumTurningRadius(obj, radius);
            obj.InternalMinTurningRadius = radius;

        end

        function MinTurningRadius = get.MinTurningRadius(obj)
        %get.MinTurningRadius Getter for property Minimum Turning Radius

            MinTurningRadius = obj.InternalMinTurningRadius;

        end

        function set.MotionPrimitiveLength(obj, length)
        %set.MotionPrimitiveLength Setter for property length of motion
        %   primitive length

            validateMotionPrimitiveLength(obj, length);
            obj.InternalMotionPrimitiveLength = length;

        end

        function MotionPrimitiveLength = get.MotionPrimitiveLength(obj)
        %get.MotionPrimitiveLength Getter for property length of motion
        %   primitive length

            MotionPrimitiveLength = obj.InternalMotionPrimitiveLength;

        end

        function set.NumMotionPrimitives(obj, numPrimitives)
        %set.NumMotionPrimitives Setter for property number of motion
        %   primitives

            validateattributes(numPrimitives, {'single', 'double'}, ...
                               {'nonempty', 'scalar', 'finite', 'integer', '>=', 3, ...
                                'odd'}, 'plannerHybridAStar', 'NumMotionPrimitives');
            obj.InternalNumMotionPrimitives = numPrimitives;

        end

        function NumMotionPrimitives = get.NumMotionPrimitives(obj)
        %get.NumMotionPrimitives Getter for property number of motion
        %   primitives

            NumMotionPrimitives = obj.InternalNumMotionPrimitives;

        end

        function set.ForwardCost(obj, cost)
        %set.ForwardCost Setter for property forward cost

            validateattributes(cost, {'single', 'double'}, ...
                               {'nonempty', 'scalar', 'finite', 'real', '>=', 1}, ...
                               'plannerHybridAStar', 'ForwardCost');
            obj.InternalForwardCost = cost;

        end

        function ForwardCost = get.ForwardCost(obj)
        %get.ForwardCost Getter for property forward cost

            ForwardCost = obj.InternalForwardCost;

        end

        function set.ReverseCost(obj, cost)
        %set.ReverseCost Setter for property reverse cost

            validateattributes(cost, {'single', 'double'}, ...
                               {'nonempty', 'scalar', 'finite', 'real', '>=', 1}, ...
                               'plannerHybridAStar', 'ReverseCost');
            obj.InternalReverseCost = cost;

        end

        function ReverseCost = get.ReverseCost(obj)
        %get.ReverseCost Getter for property reverse cost

            ReverseCost = obj.InternalReverseCost;

        end

        function set.DirectionSwitchingCost(obj, cost)
        %set.DirectionSwitchingCost Setter for property direction switching cost

            validateattributes(cost, {'single', 'double'}, ...
                               {'nonempty', 'scalar', 'finite', 'real', 'nonnegative'}, ...
                               'plannerHybridAStar', 'DirectionSwitchingCost');
            obj.InternalDirectionSwitchingCost = cost;

        end

        function DirectionSwitchingCost = get.DirectionSwitchingCost(obj)
        %get.DirectionSwitchingCost Getter for property direction switching cost

            DirectionSwitchingCost = obj.InternalDirectionSwitchingCost;

        end

        function set.AnalyticExpansionInterval(obj, numNodes)
        %set.AnalyticExpansionInterval Setter for property analytic
        %   expansion interval

            validateattributes(numNodes, {'single', 'double'}, ...
                               {'nonempty', 'scalar', 'finite', 'integer', 'positive'}, ...
                               'plannerHybridAStar', 'AnalyticExpansionInterval');
            obj.InternalAnalyticExpansionInterval = numNodes;

        end

        function AnalyticExpansionInterval = get.AnalyticExpansionInterval(obj)
        %get.AnalyticExpansionInterval Getter for property analytic
        %   expansion interval

            AnalyticExpansionInterval = obj.InternalAnalyticExpansionInterval;

        end

        function set.InterpolationDistance(obj, stepSize)
        %set.InterpolationDistance Setter for interpolation distance

            validateattributes(stepSize, {'single', 'double'}, ...
                               {'nonempty', 'scalar', 'finite', 'real', 'positive'}, ...
                               'plannerHybridAStar', 'InterpolationDistance');
            obj.InternalInterpolationDistance = stepSize;

        end

        function InterpolationDistance = get.InterpolationDistance(obj)
        %get.InterpolationDistance Getter for interpolation distance

            InterpolationDistance = obj.InternalInterpolationDistance;

        end

    end

end
