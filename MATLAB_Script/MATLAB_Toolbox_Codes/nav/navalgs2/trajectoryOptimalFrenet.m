classdef trajectoryOptimalFrenet < handle & nav.algs.internal.InternalAccess
%trajectoryOptimalFrenet Find optimal trajectory along reference path
%
%   The trajectoryOptimalFrenet object is a path planner which samples and 
%   evaluates local trajectories based on a reference path. The planner 
%   generates a set of terminal states based on the reference path and 
%   other parameters in the object. The planner then connects the start
%   state to each terminal state using 4th or 5th order polynomials.
%   Sampled trajectories are evaluated for kinematic feasibility,
%   collision, and cost to chose an optimal path.
%
%   trajectoryOptimalFrenet(WAYPOINTS, VALIDATOR) creates a 
%   trajectoryOptimalFrenet object for a given set of WAYPOINTS and 
%   VALIDATOR. WAYPOINTS must be an N-by-2 matrix of [x y] points, and
%   VALIDATOR is a state validator object, like validatorOccupancyMap.
%
%   planner = trajectoryOptimalFrenet(___, "Name", Value, ...) sets 
%   additional parameters using name-value pairs. Multiple name-value pairs
%   can be specified in any order.
%
%       Time                   - Weight for time cost
%       ArcLength              - Weight for arc length cost
%       Deviation              - Weight for deviation from REFPATH
%       LateralSmoothness      - Weight for lateral jerk cost
%       LongitudinalSmoothness - Weight for longitudinal jerk cost
%       MaxCurvature           - Maximum feasible curvature
%       MaxAcceleration        - Maximum acceleration
%       CostFunction           - User defined cost function handle
%       TimeResolution         - Trajectory discretization interval
%
%   trajectoryOptimalFrenet properties:
%
%       Weights               - Weights for all trajectory costs
%       FeasibilityParameters - Structure containing feasibility parameters
%       TimeResolution        - Trajectory discretization interval
%       CostFunction          - User defined cost function handle
%       TrajectoryList        - List of all possible trajectories
%       TerminalStates        - List of goal states
%       Waypoints             - Waypoints of reference path
%       NumSegments           - Number of longitudinal segments for each 
%                               trajectory
%       DeviationOffset       - Offset of deviation from reference path
%
%   trajectoryOptimalFrenet methods:
%       cart2frenet          - Convert from Cartesian to Frenet frame
%       copy                 - Creates a deep copy of the object
%       frenet2cart          - Convert from Frenet frame to Cartesian frame
%       plan                 - Plan optimal trajectory
%       show                 - Visualize trajectories
%
%   Example:
%     % Create a state validator object
%     validator = validatorOccupancyMap;
%
%     % Create a binary occupancy map
%     map = binaryOccupancyMap(500,500);
%
%     % Assign map and state bounds to the validator object
%     validator.Map = map;
%     validator.StateSpace.StateBounds(1:2,:) = [map.XWorldLimits; ...
%                                                map.YWorldLimits];
%
%     % Create a reference path
%     refPath = [100,100;400,400];
%
%     % Initialize planner object with reference path array and
%     % stateValidator
%     planner = trajectoryOptimalFrenet(refPath, validator);
%
%     % Initial state of vehicle [x y theta kappa speed acceleration]
%     initState = [110 110 pi/3 0 0 0];
%
%     % Convert to Frenet state [s ds/dt d2s/dt2 l dl/ds d2l/ds2]
%     initFrenetState = cart2frenet(planner, initState);
%
%     % Plan from initial Frenet state
%     trajectory = plan(planner, initFrenetState);
%
%     % Visualize the trajectory using show function
%     show(planner,"TrajectoryColor","velocity")
%
%   See also referencePathFrenet, validatorOccupancyMap
%
%   References:
%
%   [1] M. Werling, J. Ziegler, S. Kammel, and S. Thrun. "Optimal
%       trajectory generation for dynamic street scenarios in a frenet
%       frame." In 2010 IEEE International Conference on Robotics and
%       Automation, 2010, pp. 987-993.
%

%   Copyright 2019-2020 The MathWorks, Inc.

%#codegen
    properties
        %Weights Weights for all trajectory costs
        %   The Weights property is a structure containing scalars for the
        %   cost multipliers of the corresponding trajectory attributes.
        %   The total trajectory cost is a sum of all attributes multiplied
        %   by their weights.
        %   The fields of the structure are:
        %
        %   Time: Weight for time cost. The total time taken to reach
        %   the terminal state in seconds.
        %   Default: 0
        %
        %   ArcLength: Weight for arc length cost. The cost function
        %   multiplies the weight by total length of the generated
        %   trajectories in meters.
        %   Default: 0
        %
        %   LateralSmoothness: Weight for lateral jerk cost. The cost
        %   function multiplies the weight by integral of lateral jerk
        %   squared. This value determines the aggressiveness of the
        %   trajectory in the lateral direction (perpendicular to the
        %   reference path), specified as a scalar. To penalize lateral
        %   jerk in the planned trajectory, increase this cost value.
        %   Default: 0
        %
        %   LongitudinalSmoothness: Weight for longitudinal jerk cost. The
        %   cost function multiplies the weight by the integral of
        %   longitudinal jerk squared. This value determines the
        %   aggressiveness of the trajectories in the longitudinal
        %   direction (direction of the reference path). To penalize large
        %   change in forward and backward acceleration increase this value.
        %   Default: 0
        %
        %   Deviation: Weight for deviation from REFPATH. The cost function
        %   multiplies the weight by perpendicular distance from the
        %   reference path at the end of the trajectory in meters.
        %   Default: 1
        Weights = struct('Time',0,'Deviation',1,'ArcLength',0,'LateralSmoothness',0,'LongitudinalSmoothness',1);

        %FeasibilityParameters Structure containing feasibility parameters
        %
        %   The Feasibility Property is a structure containing scalar
        %   values to check the validity of a trajectory. Each field of the
        %   structure corresponds to a trajectory parameter:
        %
        %   MaxCurvature: Maximum curvature that can be executed by the
        %   vehicle specified in m^-1. This value determines the kinematic
        %   feasibility of the trajectory.
        %   Default: 0.1
        %
        %   MaxAcceleration: Maximum acceleration in the direction of
        %   motion of the vehicle specified in m/s^2. Decrease this value
        %   to lower the limit on the acceleration of the vehicle in the
        %   forward or reverse direction.
        %   Default: 2.5
        FeasibilityParameters = struct('MaxCurvature',0.1,'MaxAcceleration', 2.5);

        %TimeResolution Trajectory discretization interval
        %   Time interval between discretized states of the trajectory,
        %   specified in seconds. The state validity and cost function are
        %   based on these discretized states.
        %   Default: 0.1
        TimeResolution = 0.1

        %NumSegments Number of longitudinal segments for each trajectory
        %   Number of partitions of the longitudinal terminal states. This
        %   is used for generating intermediate longitudinal terminal
        %   states to which all lateral terminal states are combined with
        %   for generating more motion primitives to each terminal state.
        %   For example, setting the property to 2 creates two partitions
        %   between each longitudinal terminal state. Trajectories are
        %   generated to reach the intermediate longitudinal states with
        %   all the given lateral terminal states.
        %   Default: 1
        NumSegments = 1

        %DeviationOffset Offset of deviation from reference path
        %   Offset for the deviation from the reference path in the lateral
        %   direction. In the Frenet coordinate system, negative values
        %   indicate a deviation to the right (positive = left). Set this
        %   property to bias your solution to a certain turn direction when
        %   avoiding obstacles in the reference path.
        %   Default: 0
        DeviationOffset = 0
    end

    properties(SetAccess = immutable)

        %CostFunction User defined cost function handle
        %   Function handle that accepts an array of N-by-7 states,
        %   TRAJSTATES for each trajectory and returns a cost value as a
        %   scalar. The PLAN function returns the path with the lowest cost.
        %   The cost function signature is cost = COSTFUNCTION(TRAJSTATES).
        %   Default: nullCost
        CostFunction
    end

    properties(SetAccess = private)

        %TrajectoryList List of all possible trajectories
        %   The TrajectoryList property is a structure array containing all
        %   trajectories and their corresponding parameters. The fields of
        %   the structure are: Trajectory, Cost, MaxAcceleration,
        %   MaxCurvature and Feasible. The Feasibility field is a 4-element
        %   vector indicating the validity of the path based on four
        %   parameters, in this order: velocity, acceleration, curvature
        %   and collision.A value of 1 means the trajectory is valid, 0
        %   means invalid and -1 means not checked.
        %   Default: []
        TrajectoryList
    end
    properties(Dependent)

        %TerminalStates A structure of all the goal states
        %   The TerminalStates property contains a list of goal states
        %   relative to the reference path. These parameters define the
        %   sampling behavior for generating alternative trajectory
        %   segments between START and each GOAL state. The fields are:
        %
        %   Longitudinal: Array of lengths of the trajectory segment,
        %   specified in meters.
        %   Default: 30:15:90
        %
        %   Lateral: Array of deviations from the reference path in the
        %   perpendicular direction at the goal state specified in meters.
        %   Default: -2:1:2
        %
        %   Speed: Velocity at the goal state in the direction of motion,
        %   specified as a scalar specified in m/s.
        %   Default: 10
        %
        %   Acceleration: Acceleration at the goal state in the direction
        %   of motion specified in m/s^2.
        %   Default: 0
        %
        %   Time: Array of end-times for executing the trajectory segment
        %   specified in seconds.
        %   Default: 7
        TerminalStates

        %Waypoints Waypoints of reference path
        %   The Waypoints property is an N-by-2 array of [x y] waypoints,
        %   which act as a reference for planning alternative trajectories
        %   that are optimized by this planner.
        %   Default: []
        Waypoints
    end
    properties (Access={?trajectoryOptimalFrenet, ?nav.algs.internal.InternalAccess})
        %SamplerObject Class object for sampling lattices
        %   Consists of methods for generating lattices (motion primitives)
        %   which are continuous functions joining discrete states.
        %   Default: []
        SamplerObject

        %ReferencePathObject Reference path to be tracked for local planning
        %   Class object of type ReferencePath consisting of methods to
        %   query reference point closest to the current state and
        %   performing interpolation to estimate lateral and longitudinal
        %   deviation from it.
        %   Default: []
        ReferencePathObject

        %StateValidator State validator for validating trajectories
        %   The StateValidator property is specified as a
        %   validatorOccupancyMap object. The validator checks trajectories
        %   for collisions with the specified occupancy map.
        %   state validator object like validatorOccupancyMap, or another
        %   implementation of the nav.StateValidator class. The validator
        %   checks trajectories for state validity.
        %   nav.StateValidator object. The validator checks trajectories
        %   for state validity.
        %   Default: []
        StateValidator

        %BestTrajectoryIndex Index of the optimal trajectory
        %   Default: nan
        BestTrajectoryIndex = nan

        %BestTrajectoryCost Cost of the optimal trajectory
        %   Default: inf
        BestTrajectoryCost = inf

        %InternalTerminalStates Dependent property TerminalStates
        %   Internal data storage for dependent property TerminalStates
        InternalTerminalStates

        %optimalTrajHandle Graphics handle representing the optimal trajectory
        optimalTrajHandle

        %trajectoriesHandle Graphics handle representing all generated trajectories
        trajectoriesHandle
    end
    properties(Access=private,Constant)

        % Create a default name value pairs for constructor properties
        DefaultConstructorNames = { 'Time', ...
                            'Deviation', ...
                            'ArcLength', ...
                            'LateralSmoothness', ...
                            'LongitudinalSmoothness', ...
                            'MaxCurvature', ...
                            'MaxAcceleration', ...
                            'CostFunction',...
                            'TimeResolution'...
                            'NumSegments',...
                            'DeviationOffset'};

        DefaultConstructorValues  = { 0, ...
                            1, ...
                            0, ...
                            0, ...
                            0, ...
                            0.1, ...
                            2.5, ...
                            [],...
                            0.1,...
                            1,...
                            0};

        %MaxNumTrajectoryThreshold Issue warning beyond this threshold
        %   Threshold for issuing a warning about the number of
        %   trajectories calculated based on algorithm performance. The
        %   default value is based on the average number trajectory
        %   evaluations possible withing 0.5s of calling plan.
        MaxNumTrajectoryThreshold = 1500
    end

    methods
        function obj = trajectoryOptimalFrenet(refPath, stateValidator, varargin)
        %trajectoryOptimalFrenet Construct an instance of the planner

            % Check for number of input arguments
            narginchk(2,inf);

            if isa(refPath,'referencePathFrenet')
                obj.ReferencePathObject = refPath;
            else
                % Initialize reference path object
                obj.ReferencePathObject = nav.algs.internal.FrenetReferencePath(refPath);
            end

            % Validate the state validator attributes
            validateattributes(stateValidator, {'nav.StateValidator'},{'nonempty','scalar'},'trajectoryOptimalFrenet','validator')
            if isa(stateValidator,'validatorOccupancyMap')
                nav.internal.validation.validateValidatorOccupancyMap(stateValidator, "trajectoryOptimalFrenet", 'validator');
            else
                % For custom state validators throw an error if number of
                % state variables in the state space is greater than 7
                coder.internal.errorIf(stateValidator.StateSpace.NumStateVariables > 7,...
                                       'nav:navalgs:trajectoryoptimalfrenet:InvalidStateValidator')
            end

            % Initialize state validator object
            obj.StateValidator = stateValidator;

            % Create name-value parser
            parser = robotics.core.internal.NameValueParser(...
                obj.DefaultConstructorNames, obj.DefaultConstructorValues );
            parse(parser, varargin{:});

            CostFunction = parameterValue(parser, 'CostFunction');
            % Set the immutable property CostFunction
            if(~isempty(CostFunction))
                trajectoryOptimalFrenet.validateCostFunction(CostFunction)
                obj.CostFunction = CostFunction;
            else
                obj.CostFunction = @trajectoryOptimalFrenet.nullCost;
            end

            % Parse name-value pairs
            obj.parameterParserConstructor(parser);
        end

        function [trajectory, index, cost, exitFlag] = plan(obj, startState)
        %plan Plan optimal trajectory
        %   [TRAJ,INDEX,COST,FLAG] = plan(PLANNER,START) computes a
        %   trajectory, TRAJ, from a list candidate trajectories
        %   generated from the trajectoryOptimalFrenet object, PLANNER.
        %   START is specified as a 6-element vector of Frenet states s,
        %   ds/dt, d2s/dt2, l, dl/ds and d2l/ds2 where s is arc length from
        %   the first point in reference path and l is lateral distance from
        %   the closest point at s on the reference path. The output
        %   trajectory, TRAJ, also has an associated COST and INDEX for the
        %   TrajectoryList property of the planner.
        %   FLAG is a numeric exit flag indicating status of the solution:
        %      0 - Optimal trajectory was found.
        %      1 - No feasible trajectory exists.
        %
        %   When no feasible trajectory exists, the planner returns an
        %   empty trajectory. Modify the parameters of the PLANNER object
        %   to improve results of the planning algorithm.
        %
        %   Example:
        %     % Create a state validator object
        %     validator = validatorOccupancyMap;
        %
        %     % Create a binary occupancy map
        %     map = binaryOccupancyMap(500,500);
        %
        %     % Assign map and state bounds to the validator object
        %     validator.Map = map;
        %     validator.StateSpace.StateBounds(1:2,:) = [map.XWorldLimits; ...
        %                                                map.YWorldLimits];
        %
        %     % Create a reference path
        %     refPath = [100,100;400,400];
        %
        %     % initialize planner object with reference path array and
        %     % stateValidator
        %     planner = trajectoryOptimalFrenet(refPath, validator);
        %
        %     % Initial state of vehicle [x y theta kappa speed acceleration]
        %     initState = [110 110 pi/3 0 0 0];
        %
        %     % Convert to Frenet state [s ds/dt d2s/dt2 l dl/ds d2l/ds2]
        %     initFrenetState = cart2frenet(planner, initState);
        %
        %     % Plan from initial Frenet state
        %     trajectory = plan(planner, initFrenetState);
        %
        %     % Visualize the trajectory using show function
        %     show(planner,"Trajectory","all","TrajectoryColor","velocity")
        %
        %   See also trajectoryOptimalFrenet, show

        % Check for number of input arguments
            narginchk(2,2)

            % Find the maximum arc length from the reference path
            sRefMax = obj.ReferencePathObject.Length;

            % Find the maximum planning horizon from the terminal states
            terminalSMax = max(obj.TerminalStates.Longitudinal);

            % Validate input argument attributes
            startState = robotics.internal.validation.validateTrajectoryState(...
                startState,'plan','startState');

            validateattributes(startState(1), {'numeric'},{'nonnegative'},'plan','startState(1)')
            validateattributes(startState(2), {'numeric'},{'nonnegative'},'plan','startState(2)')
            validateattributes(startState(3), {'numeric'},{'<=',obj.FeasibilityParameters.MaxAcceleration},...
                               'plan','startState(3)')

            % Check the curvature feasibility of the startState from the
            % reference path i.e 1 - kappaReference * l > 0
            if (startState(1) + terminalSMax) > sRefMax + sqrt(eps)
                coder.internal.error('nav:navalgs:trajectoryoptimalfrenet:PlanningHorizonExceeded')
            end
            % Convert input Frenet states to Cartesian
            startStateCartesian = obj.frenet2cart(startState);

            % Append 0 for time state
            startStateCartesian = [startStateCartesian,0];

            % Initialize variables
            obj.BestTrajectoryIndex = nan(1,1);
            obj.BestTrajectoryCost = inf;

            % Check if the terminal state is not beyond the reference path
            % with tolerance of sqrt(eps)
            coder.internal.errorIf((startState(1) + terminalSMax) > sRefMax + sqrt(eps), ...
                                   'nav:navalgs:trajectoryoptimalfrenet:PlanningHorizonExceeded')

            % Check if initial state is in the collision free space or not
            coder.internal.errorIf(any(obj.StateValidator.isStateValid(startStateCartesian(1:obj.StateValidator.StateSpace.NumStateVariables)) == 0,'all'), ...
                                   'nav:navalgs:trajectoryoptimalfrenet:StateCollisionFree')

            totalTrajectories = size(obj.TerminalStates.Longitudinal,2) * size(obj.TerminalStates.Time,2)...
                * size(obj.TerminalStates.Lateral,2)^(obj.NumSegments);

            % Throw a warning when total number of trajectories being
            % evaluated are more than the threshold
            if totalTrajectories > obj.MaxNumTrajectoryThreshold
                coder.internal.warning('nav:navalgs:trajectoryoptimalfrenet:NumTrajectoriesHigh',totalTrajectories)
            end

            % Compute candidate trajectories from startState to all the
            % terminal states
            obj.TrajectoryList = obj.SamplerObject.generateTrajectories(startState);

            % Evaluate total cost by adding value returned by CostFunction
            costs = zeros(numel(obj.TrajectoryList),1);
            for i = 1:numel(obj.TrajectoryList)
                obj.TrajectoryList(i).Cost = obj.TrajectoryList(i).Cost + obj.CostFunction(obj.TrajectoryList(i).Trajectory);
                costs(i) = obj.TrajectoryList(i).Cost;
            end
            % Sort all the trajectories based on cost
            [costs, indexes] = sort(costs);

            % Allocate variable for feasibility
            feasibilityConditions = ones(length(obj.TrajectoryList),4);

            % Set feasibility to -1 for collision feasibility element
            feasibilityConditions(:,4) = -1;

            % Check all trajectories for feasibility bounds
            for j = 1:length(obj.TrajectoryList)
                trajectory = obj.TrajectoryList(indexes(j)).Trajectory;
                if ~isempty(trajectory)
                    % Assign nan cost to trajectories with negative speed
                    if ~isequal(trajectory(:,5) >= 0,ones(size(trajectory,1),1))
                        feasibilityConditions(j,1) = 0;
                        obj.TrajectoryList(indexes(j)).Cost = nan;
                    end

                    % Check the trajectory for maximum acceleration
                    feasibilityConditions(j,2) = obj.TrajectoryList(indexes(j)).MaxAcceleration <= ...
                        obj.FeasibilityParameters.MaxAcceleration;

                    % Check the trajectory for maximum curvature
                    feasibilityConditions(j,3) = obj.TrajectoryList(indexes(j)).MaxCurvature <= ...
                        obj.FeasibilityParameters.MaxCurvature;

                    obj.TrajectoryList(indexes(j)).Feasible = feasibilityConditions(j,:);

                else
                    % In case of empty trajectory set the feasibility array
                    % to zero
                    feasibilityConditions(j,:) = [0 0 0 -1];
                end
            end
            % Check all trajectories using state validator in ascending
            % order of costs and break as soon as a feasible trajectory is
            % found
            for j = 1:length(obj.TrajectoryList)
                trajectory = obj.TrajectoryList(indexes(j)).Trajectory;

                % If trajectory is feasible according to all the
                % parameters then process
                if isequal(feasibilityConditions(j,1:3),ones(1,3))

                    % Check for collision at all states of the
                    % trajectory
                    isValid = obj.StateValidator.isStateValid(trajectory(:,1:obj.StateValidator.StateSpace.NumStateVariables));
                    if any(~isValid)
                        obj.TrajectoryList(indexes(j)).Feasible(4) = false;
                    else
                        obj.BestTrajectoryIndex = indexes(j);
                        obj.BestTrajectoryCost = costs(j);
                        obj.TrajectoryList(indexes(j)).Feasible(4) = true;
                        break
                    end
                end
            end

            index = obj.BestTrajectoryIndex;
            cost = obj.BestTrajectoryCost;
            % Return empty if no feasible trajectory is found
            if ~isfinite(obj.BestTrajectoryIndex)
                % No feasible paths found, return empty trajectory
                trajectory = [];
                exitFlag = 1;
            else
                % Update output trajectory and error flag
                trajectory = obj.TrajectoryList(index(1)).Trajectory;
                exitFlag = 0;
            end
        end



        function axHandle = show(obj, varargin)
        %show Visualize trajectories
        %   show(planner) visualizes the reference path and trajectory
        %   from the candidates generated by the PLAN function. The
        %   trajectory is shown as a line plot. Plot also includes datatip
        %   mode which can be used to visualize the feasibility vector and
        %   and index of the trajectory from TrajectoryList property.
        %
        %   axisHandle = show(planner) returns the handle of the axes used
        %   by the show function.
        %
        %   show(PLANNER,Name,Value) provides additional options specified
        %   by one or more Name,Value pair arguments. Name must appear
        %   inside double quotes (""). You can specify several name-value
        %   pair arguments in any order as Name1,Value1,...,NameN,ValueN:
        %
        %       "Parent"        - Axis handle to be used by show function
        %
        %       "Trajectory"    - A string to select display of either
        %                         "optimal" or "all" trajectories
        %
        %                         Default: "optimal"
        %
        %       "ReferencePath" - A string to turn on or off the display of
        %                         reference path.
        %
        %                         Default: "on"
        %
        %       "TrajectoryColor"- A string to determine which values of the
        %                         trajectory to display as a color-gradient
        %                         along the path, specified as
        %                         "acceleration", "cost", "velocity", and
        %                         "none".
        %
        %                         Default: "velocity"
        %   Example:
        %     % Create a state validator object
        %     validator = validatorOccupancyMap;
        %
        %     % Create a binary occupancy map
        %     map = binaryOccupancyMap(500,500);
        %
        %     % Assign map and state bounds to the validator object
        %     validator.Map = map;
        %     validator.StateSpace.StateBounds(1:2,:) = [map.XWorldLimits; ...
        %                                                map.YWorldLimits];
        %
        %     % Create a reference path
        %     refPath = [100,100;400,400];
        %
        %     % initialize planner object with reference path array and
        %     % stateValidator
        %     planner = trajectoryOptimalFrenet(refPath, validator);
        %
        %     % Initial state of vehicle [x y theta kappa speed acceleration]
        %     initState = [110 110 pi/3 0 0 0];
        %
        %     % Convert to Frenet state [s ds/dt d2s/dt2 l dl/ds d2l/ds2]
        %     initFrenetState = cart2frenet(planner, initState);
        %
        %     % Plan from initial Frenet state
        %     trajectory = plan(planner, initFrenetState);
        %
        %     % Visualize the trajectory using show function
        %     show(planner,"Trajectory","all")
        %
        %   See also trajectoryOptimalFrenet, plan

        % If called in code generation, throw incompatibility error
            coder.internal.errorIf(~coder.target('MATLAB'), ...
                                   'nav:navalgs:trajectoryOptimalFrenet:GraphicsSupportCodegen', 'show')

            % Check for number of input arguments
            narginchk(1,inf);

            % Initialize with a default name-value pair
            defaultNames = {'Parent', ...
                            'Trajectory', ...
                            'ReferencePath', ...
                            'TrajectoryColor'};
            defaultValues = {[], ...
                             'optimal', ...
                             'on', ...
                             'none'};
            defaultTypes = {{'matlab.graphics.axis.Axes'},...
                            {'char','string'},...
                            {'char','string'},...
                            {'char','string'}};
            parser = robotics.core.internal.NameValueParser(defaultNames, defaultValues);
            parse(parser, varargin{:});
            for index = 2:numel(defaultNames)
                validateattributes(parameterValue(parser, defaultNames{index}), ...
                                   defaultTypes{index}, {'nonempty'},'show',defaultNames{index})
            end
            axisHandle = parameterValue(parser, 'Parent');

            % Create a new axes if not assigned
            if isempty(axisHandle)
                axisHandle = newplot;
            else
                robotics.internal.validation.validateAxesUIAxesHandle(axisHandle);
            end

            trajectoryPlotType = validatestring(parameterValue(parser, 'Trajectory'),...
                                                {'optimal','all'});

            colorParsed = validatestring(parameterValue(parser, 'TrajectoryColor'),...
                                         {'velocity','acceleration','cost','none'});

            isReferencePathPlot = validatestring(parameterValue(parser, 'ReferencePath'),...
                                                 {'on','off'});

            % Plot the reference path
            if strcmp(isReferencePathPlot , 'on')
                % Call show on the reference path to display the reference
                % path
                obj.ReferencePathObject.show('Parent',axisHandle);
            else
                % If the reference path is currently shown on the plot,
                % hide it. If it is not shown, we don't need to do anything
                if ~isempty(obj.ReferencePathObject.PathHandle)
                    if isvalid(obj.ReferencePathObject.PathHandle)
                        set(obj.ReferencePathObject.WaypointsHandle, 'Visible', 'off');
                        set(obj.ReferencePathObject.PathHandle,   'Visible', 'off');
                    end
                end
            end

            hold(axisHandle, 'on')
            handleCleanup = onCleanup(@() hold(axisHandle, "off"));

            % Get all child objects of the plot
            children = axisHandle.Children;

            % Plot the trajectories
            switch trajectoryPlotType
              case 'optimal'
                if ~isempty(obj.trajectoriesHandle)
                    if isvalid(obj.trajectoriesHandle)
                        % Hide non-optimal trajectories if user previously
                        % plotted them, but does not want them shown during
                        % this call
                        set(obj.trajectoriesHandle,'Visible','off');
                    end
                end
              case 'all'
                % Plot all the trajectories in TrajectoryList with finite cost
                % Collect all xy trajectory data into single array,
                % separated by nans
                trajectoryCellArray  = {obj.TrajectoryList.Trajectory};

                trajectorySizes = cellfun(@(x)size(x,1),trajectoryCellArray);

                trajectoryGroup = [trajectoryCellArray; ...
                                   repelem({nan(1,7)},1,length(obj.TrajectoryList))];

                trajectoryData = vertcat(trajectoryGroup{:});

                % Get color for trajectories
                cData = obj.getColorMap(trajectoryData, trajectorySizes, colorParsed);

                % Plot all trajectory data in a single handle
                if isempty(obj.trajectoriesHandle) || ~any(obj.trajectoriesHandle == children)
                    % If handle is not found on the current axis, plot a
                    % new line and overwrite the line handle
                    obj.trajectoriesHandle = patch(trajectoryData(:,1), trajectoryData(:,2),...
                                                   cData, 'Parent', axisHandle, 'EdgeColor','flat',...
                                                   'FaceColor','none', 'LineWidth',1,'LineStyle','-',...
                                                   'Tag','Candidate Trajectories');
                    datatipObject = datatip(obj.trajectoriesHandle);
                    datatipObject.Visible = 'off';
                else
                    % Otherwise, just update the existing handle's data
                    set(obj.trajectoriesHandle, 'XData', trajectoryData(:,1),...
                                      'YData', trajectoryData(:,2), 'CData', cData, 'Visible', 'on');
                end
                obj.updateDataTip(trajectorySizes);
            end

            % Plot the optimal trajectory at BestTrajectoryIndex
            if isfinite(obj.BestTrajectoryCost)
                trajectory = [obj.TrajectoryList(obj.BestTrajectoryIndex).Trajectory;nan(1,7)];

                if strcmp(colorParsed,'cost')
                    costs = normalize([obj.TrajectoryList.Cost],'range');
                    cData = repelem(costs(obj.BestTrajectoryIndex),size(trajectory,1));
                else
                    cData = obj.getColorMap(trajectory, 0, colorParsed);
                end
                if isempty(obj.optimalTrajHandle) || ~any(obj.optimalTrajHandle == children)
                    % If handle is not found on the current axis, plot a
                    % new line and overwrite the line handle
                    obj.optimalTrajHandle = patch(trajectory(:,1), trajectory(:,2), ...
                                                  cData, 'Parent', axisHandle, 'EdgeColor','flat',...
                                                  'FaceColor','none','LineWidth',3,'LineStyle','-',...
                                                  'Tag','Optimal Trajectory');
                else
                    % Otherwise, just update the existing handle's data
                    set(obj.optimalTrajHandle, 'XData', trajectory(:,1), ...
                                      'YData', trajectory(:,2),'CData', cData);
                end
                if strcmp(trajectoryPlotType,'all')
                    % Set optimal Trajectory Color to Black if all
                    % candidates are plotted
                    set(obj.optimalTrajHandle,'EdgeColor','black','FaceColor','none');
                end

                % Plot legend
                if strcmp(isReferencePathPlot , 'on') && ~isempty(obj.ReferencePathObject.PathHandle)
                    legend([obj.ReferencePathObject.WaypointsHandle, obj.ReferencePathObject.PathHandle, obj.optimalTrajHandle],...
                           message('nav:navalgs:trajectoryoptimalfrenet:WaypointsLegend').getString,...
                           message('nav:navalgs:trajectoryoptimalfrenet:ReferencePathLegend').getString,...
                           message('nav:navalgs:trajectoryoptimalfrenet:OptimalTrajectoryLegend').getString)
                else
                    legend(obj.optimalTrajHandle,message('nav:navalgs:trajectoryoptimalfrenet:OptimalTrajectoryLegend').getString)
                end
            else
                if ~isempty(obj.optimalTrajHandle)
                    if isvalid(obj.optimalTrajHandle)
                        % Hide previous optimal trajectory
                        set(obj.optimalTrajHandle,'Visible','off');
                    end
                end
            end

            % Plot colorbar
            if ~strcmp(colorParsed,'none')
                colorbarObj = colorbar('peer',axisHandle);
                colorbarObj.Label.String = message('nav:navalgs:trajectoryoptimalfrenet:ColorBarLabel',colorParsed).getString;
            else
                colorbar('off');
            end

            % Only return handle if user requested it
            if nargout > 0
                axHandle = axisHandle;
            end
        end

        function frenetStates = cart2frenet(obj, cartesianStates)
        %cart2frenet Convert Cartesian states to Frenet states
        %  cart2frenet(planner, cartesianStates) converts 1-by-6 matrix of Cartesian
        %  states x, y, theta, kappa, speed and acceleration to a 1-by-6 matrix of Frenet
        %  states s, ds/dt, d2s/dt2, l, dl/ds and d2l/ds2 where s is arc length from the
        %  first point in reference path and l is lateral distance from the closest point
        %  at s on the reference path.
        %
        %   Example:
        %     % Create a state validator object
        %     validator = validatorOccupancyMap;
        %
        %     % Create a binary occupancy map
        %     map = binaryOccupancyMap(500,500);
        %
        %     % Assign map to validator object
        %     validator.Map = map;
        %
        %     % Create a reference path
        %     refPath = [100,100;400,400];
        %
        %     % initialize planner object with reference path array and
        %     % stateValidator
        %     planner = trajectoryOptimalFrenet(refPath, validator);
        %
        %     % Initial state of vehicle [x y theta kappa speed acceleration]
        %     cartesianStates = [110 110 pi/4 0 0 0];
        %
        %     % Convert to frenet states
        %     frenetStates = cart2frenet(planner, cartesianStates)
        %
        %     See also frenet2cart, trajectoryOptimalFrenet

        % Make states a row vector
            cartesianStates = cartesianStates(:)';

            % Find the corresponding path point on the reference path
            pathPoint = obj.ReferencePathObject.closestPoint(cartesianStates(1:2));

            % Find angle between root point on reference path and cartesianStates
            deltaTheta = robotics.internal.angdiff(pathPoint(3),cartesianStates(3));

            % Check that angle between the reference path and the Cartesian
            % state is within -pi/2 and pi/2
            coder.internal.errorIf(deltaTheta <= -pi/2 || deltaTheta >= pi/2, ...
                                   'nav:navalgs:trajectoryoptimalfrenet:DeltaThetaOutOfBounds',sprintf('%.2f',deltaTheta))

            cartesianStates = robotics.internal.validation.validateTrajectoryState(...
                cartesianStates,'cart2frenet','cartesianStates');
            frenetStates = obj.ReferencePathObject.global2frenet(cartesianStates);
        end

        function cartesianStates = frenet2cart(obj, frenetStates)
        %frenet2cart Convert Frenet states to Cartesian states
        %  frenet2cart(planner, frenetStates) converts a 1-by-6 matrix of Frenet
        %  states s, ds/dt, d2s/dt2, l, dl/ds and d2l/ds2 where s stands for station (arc
        %  length) from the first point in reference path and l is lateral distance from the
        %  closest point in reference path to a 1-by-6 matrix of cartesian states x, y,
        %  theta, kappa, speed and acceleration.
        %
        %   Example:
        %     % Create a state validator object
        %     validator = validatorOccupancyMap;
        %
        %     % Create a binary occupancy map
        %     map = binaryOccupancyMap(500,500);
        %
        %     % Assign map to validator object
        %     validator.Map = map;
        %
        %     % Create a reference path
        %     refPath = [100,100;400,400];
        %
        %     % initialize planner object with reference path array and
        %     % stateValidator
        %     planner = trajectoryOptimalFrenet(refPath, validator);
        %
        %     frenetStates = [10, 1, 0, 3, 0, 0];
        %
        %     % Convert Frenet states to Cartesian states
        %     cartesianStates = frenet2cart(planner, frenetStates)
        %
        %     See also cart2frenet, trajectoryOptimalFrenet

        % Find the corresponding path point on the reference path
            pathPoint = obj.ReferencePathObject.interpolate(frenetStates(1));

            oneMinusKappaRefL = 1 - pathPoint(4) .* frenetStates(4);

            % Check the curvature feasibility of the frenetStates from the
            % reference path i.e 1 - kappaReference * l > 0
            coder.internal.errorIf(oneMinusKappaRefL <= 0, ...
                                   'nav:navalgs:trajectoryoptimalfrenet:ExtremeCurvature','frenetStates')

            frenetStates = robotics.internal.validation.validateTrajectoryState(...
                frenetStates,'frenet2cart','frenetStates');
            cartesianStates = obj.ReferencePathObject.frenet2global(frenetStates);
        end
        
        function cpObj = copy(obj)
        %copy Creates a deep copy of the object

            if isempty(obj)
                cpObj = trajectoryOptimalFrenet.empty;
                return;
            end

            % construct new object
            cpObj = trajectoryOptimalFrenet(obj.Waypoints, obj.StateValidator, "CostFunction", obj.CostFunction);

            % Copy public non-dependent properties with setters
            cpObj.Weights = obj.Weights;
            cpObj.FeasibilityParameters = obj.FeasibilityParameters;
            cpObj.TimeResolution = obj.TimeResolution;
            cpObj.TrajectoryList = obj.TrajectoryList;
            cpObj.Waypoints = obj.Waypoints;
            cpObj.TerminalStates = obj.TerminalStates;
            cpObj.NumSegments = obj.NumSegments;
            cpObj.DeviationOffset = cpObj.DeviationOffset;

            % Copy internal properties
            cpObj.BestTrajectoryIndex = obj.BestTrajectoryIndex;
            cpObj.BestTrajectoryCost = obj.BestTrajectoryCost;
        end

        function set.TerminalStates(obj,TerminalStates)
        %set.TerminalStates Setter for terminal states

            validateattributes(TerminalStates, {'struct'},{'nonempty','scalar'},'','TerminalStates')

            % Iterate through all the fields of struct and validate
            % individually
            fieldTerminalStates = {'Longitudinal','Lateral','Time','Speed','Acceleration'};

            for i = 1:numel(fieldTerminalStates)
                % Validate field
                validateattributes(TerminalStates.(fieldTerminalStates{i}), {'numeric'},{'nonempty'},'',fieldTerminalStates{i})
                % Cast to double
                TerminalStates.(fieldTerminalStates{i}) = double(TerminalStates.(fieldTerminalStates{i}));
            end

            % Validate individual states
            validateattributes(TerminalStates.Longitudinal, {'double'},{'positive'},'','ArcLength')
            validateattributes(TerminalStates.Time, {'double'},{'positive'},'','Time')
            validateattributes(TerminalStates.Speed, {'double'},{'nonnegative','scalar'},'','Speed')
            validateattributes(TerminalStates.Acceleration, {'double'},{'<=',obj.FeasibilityParameters.MaxAcceleration, 'scalar'},'','Acceleration')

            obj.InternalTerminalStates = TerminalStates;

            % Assign terminal state for sampler object
            obj.SamplerObject.TerminalStates = TerminalStates;
        end

        function set.Weights(obj, Weights)
        %set.Weights Setter for Weights for cost functions
            validateattributes(Weights, {'struct'},{'nonempty','scalar'},'','Weights')

            templateStruct = struct('Time',0,'Deviation',0,'ArcLength',0,...
                                    'LateralSmoothness',0,'LongitudinalSmoothness',0);

            robotics.internal.validation.validateStructAgainstTemplate(Weights, ...
                                                              templateStruct,'', 'set.Weights')

            fieldWeights = fieldnames(Weights);

            % Iterate through all the fields of struct and validate
            % individually
            for i = 1:numel(fieldWeights)
                % Validate field
                validateattributes(Weights.(fieldWeights{i}), {'numeric'},{'finite','real','nonempty','scalar'},'',fieldWeights{i})
                % Cast to double
                Weights.(fieldWeights{i}) = double(Weights.(fieldWeights{i}));
            end

            % Assign the property after all the fields are validated
            obj.Weights = Weights;

            % Assign weights to sampler object
            obj.SamplerObject.Weights = Weights; %#ok<*MCSUP>
        end

        function set.FeasibilityParameters(obj, FeasibilityParameters)
        %set.FeasibilityParameters Setter for feasibility parameters
            validateattributes(FeasibilityParameters, {'struct'},{'nonempty','scalar'},'','FeasibilityParameters')


            templateStruct =  struct('MaxCurvature',0,'MaxAcceleration',0);

            robotics.internal.validation.validateStructAgainstTemplate(FeasibilityParameters, ...
                                                              templateStruct,'', 'set.FeasibilityParameters')

            fieldFeasibilityParameters = fieldnames(FeasibilityParameters);

            % Iterate through all the fields of struct and validate
            % individually
            for i = 1:numel(fieldFeasibilityParameters)
                % Validate field
                validateattributes(FeasibilityParameters.(fieldFeasibilityParameters{i}), {'double'},{'finite','real','nonempty','positive','scalar'},'',fieldFeasibilityParameters{i})
                % Cast to double
                FeasibilityParameters.(fieldFeasibilityParameters{i}) = double(FeasibilityParameters.(fieldFeasibilityParameters{i}));
            end

            % Assign the property after all the fields are validated
            obj.FeasibilityParameters = FeasibilityParameters;
        end

        function set.TimeResolution(obj, TimeResolution)
        %set.TimeResolution Setter for TimeResolution discretization

            validateattributes(TimeResolution, {'numeric'},{'finite','real','nonempty','positive','scalar'},'','TimeResolution')
            obj.TimeResolution = double(TimeResolution);

            % Assign TimeResolution to sampler object
            obj.SamplerObject.TimeResolution = TimeResolution;
        end

        function set.Waypoints(obj, refPath)
        %set.Waypoints Setter for reference path waypoints

            % Update the waypoints in the reference path
            obj.ReferencePathObject.Waypoints = refPath;
        end

        function Waypoints = get.Waypoints(obj)
        %get.Waypoints Get method for waypoints
            Waypoints = obj.ReferencePathObject.Waypoints;
        end

        function TerminalStates = get.TerminalStates(obj)
        %get.TerminalStates Get method for terminal states

            TerminalStates = obj.InternalTerminalStates;
        end

        function set.NumSegments(obj,NumSegments)
        %set.NumSegments Setter for NumSegments

            validateattributes(NumSegments, {'numeric'},{'finite','nonempty','scalar','positive','integer'},'','NumSegments')
            obj.NumSegments = double(NumSegments);

            % Assign NumSegments to sampler object
            obj.SamplerObject.NumSegments = NumSegments;
        end

        function set.DeviationOffset(obj,DeviationOffset)
        %set.DeviationOffset Setter for DeviationOffset

            validateattributes(DeviationOffset, {'numeric'},{'finite','real','nonempty','scalar'},'','DeviationOffset')
            obj.DeviationOffset = double(DeviationOffset);

            % Assign DeviationOffset to sampler object
            obj.SamplerObject.DeviationOffset = DeviationOffset;
        end
    end

    methods(Access = private)

        function initializeSamplerObject(obj)
        %initializeSamplerObject Update sampler using dependent properties
            obj.SamplerObject =  nav.algs.internal.FrenetSampler(obj.ReferencePathObject, ...
                                                              obj.TerminalStates, obj.Weights,'TimeResolution',obj.TimeResolution,...
                                                              'NumSegments',obj.NumSegments,...
                                                              'DeviationOffset',obj.DeviationOffset);
        end

        function parameterParserConstructor(obj, parser)
        %parameterParserConstructor Parse and validate input arguments

            coder.varsize('longitudinal','lateral','time')
            longitudinal = 30:15:90;
            lateral = -2:1:2;
            time = 7;
            obj.InternalTerminalStates = struct('Longitudinal',longitudinal,'Lateral',lateral,'Time',time,'Speed',10,'Acceleration',0);

            % Assign the properties based on the parsed values
            weights = struct('Time',parameterValue(parser, 'Time'), ...
                             'Deviation', parameterValue(parser, 'Deviation'), ...
                             'ArcLength', parameterValue(parser, 'ArcLength'), ...
                             'LateralSmoothness', parameterValue(parser, 'LateralSmoothness'), ...
                             'LongitudinalSmoothness', parameterValue(parser, 'LongitudinalSmoothness'));

            obj.SamplerObject =  nav.algs.internal.FrenetSampler(obj.ReferencePathObject, ...
                                                              obj.TerminalStates, weights);

            obj.Weights = weights;

            obj.TimeResolution = parameterValue(parser, 'TimeResolution');


            obj.FeasibilityParameters =  struct('MaxCurvature',parameterValue(parser, 'MaxCurvature'),...
                                                'MaxAcceleration', parameterValue(parser, 'MaxAcceleration'));

            % Define TrajectoryList property with variable size struct
            % array for codegen

            coder.varsize('trajectoryList','trajectory');
            trajectory = zeros(10,7);
            trajectoryList = struct('Trajectory',trajectory,...
                                    'Cost',0,...
                                    'MaxAcceleration',0,...
                                    'MaxCurvature',0,...
                                    'Feasible',[0 0 0 0]);
            obj.TrajectoryList = trajectoryList;

            obj.NumSegments = parameterValue(parser, 'NumSegments');

            obj.DeviationOffset = parameterValue(parser, 'DeviationOffset');
        end

        function cmapData = getColorMap(obj, trajectory, trajectorySizes, colorParsed)
        %getColorMap Get colormap of the plot based on colorParsed state

            switch colorParsed
              case 'velocity'

                cmapData = normalize(trajectory(:,5),'range');
              case 'acceleration'

                cmapData = normalize(trajectory(:,6),'range');
              case 'cost'
                costs = [obj.TrajectoryList.Cost];
                cmapData = normalize(repelem(costs,trajectorySizes+1),'range');
              case 'none'
                cmapData = zeros(size(trajectory(:,1),1),1);
            end
        end

        function updateDataTip(obj, trajectorySizes)
        %updateDataTip Add custom DataTip elements for trajectories
        %   updateDataTip adds dataTipRows to the line plot of each
        %   trajectory showing the index, cost and the feasible field
        %   from the TrajectoryList property.

            obj.trajectoriesHandle.DataTipTemplate.DataTipRows = [dataTipTextRow('Index',repelem(1:numel(trajectorySizes),trajectorySizes+1));...
                                dataTipTextRow('X','XData');...
                                dataTipTextRow('Y','YData');...
                                dataTipTextRow('Cost',repelem([obj.TrajectoryList.Cost],trajectorySizes+1));...
                                dataTipTextRow('Feasible',repelem(cellfun(@(x)sprintf("%d %d %d %d",x),{obj.TrajectoryList.Feasible})',trajectorySizes+1))];
        end
    end

    methods(Static)
        function validateCostFunction(CostFunction)
        %initializeCostFunction Validate user defined CostFunction property
            validateattributes(CostFunction, {'function_handle'},{'nonempty','scalar'},'','CostFunction')

            % Check for number of input arguments for the given cost
            % function
            if(nargin(CostFunction) == 1)
                validateattributes(CostFunction(ones(7,20)), {'double'},{'nonempty','finite','real','scalar'},'','output of CostFunction')
            else
                coder.internal.error('nav:navalgs:trajectoryoptimalfrenet:CostFunctionIncorrectInputSize')
            end
        end

        function cost = nullCost(~)
        % nullCost Cost function which returns zero
            cost = 0;
        end
    end
end
