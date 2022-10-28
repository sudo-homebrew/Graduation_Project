classdef plannerBiRRT < matlabshared.planning.internal.BiRRT
    %PLANNERBIRRT Create bidirectional RRT planner for geometric planning
    %   PLANNERBIRRT object is a single-query planner that uses the
    %   bidirectional Rapidly-Exploring Random Trees (RRT) algorithm
    %   with an optional connect heuristic for increased speed.
    %
    %   The bidirectional RRT planner creates two trees with root nodes at the
    %   specified start and goal states. To extend each tree, a random
    %   state is generated and a step is taken from the nearest node
    %   based on the MAXCONNECTIONDISTANCE property. The start and goal trees
    %   alternate this extension process until both trees are connected. If the
    %   ENABLECONNECTHEURISTIC property is enabled, the extension process ignores
    %   MAXCONNECTIONDISTANCE. Invalid states or connections that collide
    %   with the environment are not added to the tree.
    %
    %   plannerBiRRT(STATESPACE, STATEVAL) creates a bidirectional RRT planner 
    %   from a state space object, STATESPACE, and a state validator object,
    %   STATEVAL. STATEVAL's state space must be the same as STATESPACE.
    %
    %   plannerBiRRT properties:
    %       StateSpace               - State space for the planner
    %       StateValidator           - State validator for the planner
    %       MaxConnectionDistance    - Max connection distance between two states
    %       MaxIterations            - Max number of iterations 
    %       MaxNumTreeNodes          - Max number of nodes in the start and goal search tree
    %       EnableConnectHeuristic   - Directly join the start and goal trees
    %       
    %   plannerBiRRT methods:
    %       plan        - Plan path between two states
    %       copy        - Create deep copy of the planner object
    %
    %   Example:
    %       % Create a state space
    %       ss = stateSpaceSE2;
    %
    %       % Create an occupancyMap-based state validator using the created state space.
    %       sv = validatorOccupancyMap(ss);
    %
    %       % Create an occupancy map from an example map and set map resolution as 10 cells/meter.
    %       load exampleMaps
    %       map = occupancyMap(ternaryMap,10);
    %       sv.Map = map;
    %
    %       % Set validation distance for the validator.
    %       sv.ValidationDistance = 0.01;
    %
    %       % Update state space bounds to be the same as map limits.
    %       ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    %
    %       % Create the path planner and increase max connection distance.
    %       planner = plannerBiRRT(ss,sv);
    %       planner.MaxConnectionDistance = 0.3;
    %
    %       % Set the start and goal states.
    %       start = [20, 10, 0];
    %       goal = [40, 40, 0];
    %
    %       % Plan a path with default settings. Set rng for repetitive result.
    %       rng(100, 'twister')
    %       [pthObj, solnInfo] = plan(planner, start, goal);
    %
    %       % Display the number of iterations taken for the tree to converge.
    %       fprintf('Number of iterations: %d\n', solnInfo.NumIterations)
    %
    %       % Visualize the results.
    %       show(map);
    %       hold on;
    %       plot(solnInfo.StartTreeData(:,1), solnInfo.StartTreeData(:,2), '.-', 'color', 'b'); % Start tree expansion
    %       plot(solnInfo.GoalTreeData(:,1), solnInfo.GoalTreeData(:,2), '.-', 'color', 'g'); % Goal tree expansion
    %       plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2) % draw path
    %       hold off;
    %
    %       % Replan the path with EnableConnectHeuristic property set to true.
    %       planner.EnableConnectHeuristic = true;
    %       [pthObj, solnInfo] = plan(planner, start, goal);
    %
    %       % Display the number of iterations taken for the tree to converge.
    %       % Observe that the number of iterations is significantly less when
    %       % compared to the EnableConnectHeuristic property is set to false.
    %       fprintf('Number of iterations: %d\n', solnInfo.NumIterations)
    %
    %       % Visualize the results.
    %       figure;
    %       show(map);
    %       hold on;
    %       plot(solnInfo.StartTreeData(:,1), solnInfo.StartTreeData(:,2), '.-', 'color', 'b'); % Start tree expansion
    %       plot(solnInfo.GoalTreeData(:,1), solnInfo.GoalTreeData(:,2), '.-', 'color', 'g'); % Goal tree expansion
    %       plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2) % draw path
    %
    %   References:
    %       [1] J. Kuffner and S. LaValle, "RRT-connect: An efficient
    %       approach to single-query path planning." In Proceedings of
    %       IEEE International Conference on Robotics and Automation.
    %       Vol.2, pp.995-1001, 2000
    %
    %   See also plannerRRT, plannerRRTStar

    %   Copyright 2020 The MathWorks, Inc.

    %#codegen

    properties (SetAccess = {?nav.algs.internal.InternalAccess})
        %StateSpace State space for the planner
        StateSpace

        %StateValidator State validator for the planner
        StateValidator
    end
    
    properties(Dependent)

        %MaxNumTreeNodes Max number of nodes in the start and goal Search tree
        %   This number does not count the root nodes of both trees
        %
        %   Default: 1e4
        MaxNumTreeNodes
    end

    properties

        %MaxIterations Max number of iterations
        %   Max number of iterations allowed to extend the start/goal
        %   tree and joining it with other
        %   
        %
        %   Default: 1e4
        MaxIterations

        %MaxConnectionDistance Maximum length of a motion to be added to tree
        %
        %   Default: 0.1
        MaxConnectionDistance
    end

    properties
        %EnableConnectHeuristic Directly join the start and goal trees.   
        %   When true, planner attempts to join start and goal tree without
        %   limitation of MaxConnectionDistance.
        %
        %   Default: false
        EnableConnectHeuristic
    end

    properties(Access=protected)
        %GoalRegionInternal - Concrete goal region of BiRRT
        GoalRegionInternal

    end

    methods
        function obj = plannerBiRRT(stateSpace, stateValidator)
            %plannerBiRRT Constructor

            obj@matlabshared.planning.internal.BiRRT(stateValidator);
            obj.validateConstructorInput(stateSpace, stateValidator);
            obj.MaxNumTreeNodes = 1e4;
            obj.MaxIterations = 1e4;
            obj.MaxConnectionDistance = 0.1;
            obj.LimitMaxNumTreeNodes = true;
        end

        function [pathObj, solnInfo] = plan(obj, startState, goalState)
            %plan Plan path between two states
            %   PATH = plan(PLANNER, STARTSTATE, GOALSTATE) returns a valid 
            %   path from the start state to the goal state as a navPath 
            %   object.
            %
            %   [PATH, SOLNINFO] = plan(PLANNER, ...) also returns SOLNINFO
            %   as a structure that contains the solution information of 
            %   the planned path.
            %
            %   SOLNINFO has the following fields:
            %
            %      IsPathFound  : Indicates whether a path is found. It 
            %                     returns true if a path is found. Otherwise,
            %                     it returns false.
            %
            %      ExitFlag     : Indicates the terminate status of the planner, returned as
            %                     1 - 'GoalReached'
            %                     2 - 'MaxIterationsReached'
            %                     3 - 'MaxNumNodesReached'
            %
            % StartTreeNumNodes : Number of nodes in the start search tree 
            %                     when the planner terminates (excluding 
            %                     the root node).
            %
            %  GoalTreeNumNodes : Number of nodes in the goal search tree 
            %                     when the planner terminates (excluding 
            %                     the root node).
            %
            %    NumIterations  : Number of iterations taken to extend the
            %                     start/goal tree and joining it with other
            %
            %    StartTreeData  : A collection of explored states that reflects
            %                     the status of the start search tree when planner
            %                     terminates. Note that nan values are inserted
            %                     as delimiters to separate each individual edge
            %
            %    GoalTreeData   : A collection of explored states that reflects
            %                     the status of the goal search tree when planner
            %                     terminates. Note that nan values are inserted
            %                     as delimiters to separate each individual edge

            % check whether start and goal states are valid
            if ~all(obj.StateValidator.isStateValid(startState)) || ~all(all(isfinite(startState)))
                coder.internal.error('nav:navalgs:plannerrrt:StartStateNotValid');
            end

            if ~all(obj.StateValidator.isStateValid(goalState)) || ~all(all(isfinite(goalState)))
                coder.internal.error('nav:navalgs:plannerrrt:GoalStateNotValid');
            end

            startState = nav.internal.validation.validateStateVector(startState, ...
                obj.StateSpace.NumStateVariables, 'plan', 'startState');
            goalState = nav.internal.validation.validateStateVector(goalState, ...
                obj.StateSpace.NumStateVariables, 'plan', 'goalState');
            
            if all(startState == goalState)
                pathObj = navPath(obj.StateSpace, [startState; goalState]);
                solnInfo = struct();
                solnInfo.IsPathFound = true;
                solnInfo.ExitFlag = obj.GoalReached;
                solnInfo.NumIterations = 0;
                solnInfo.StartTreeData = [startState;...
                    nan(1,obj.StateSpace.NumStateVariables)];
                solnInfo.GoalTreeData = [goalState;...
                    nan(1,obj.StateSpace.NumStateVariables)];
                solnInfo.StartTreeNumNodes = 0;
                solnInfo.GoalTreeNumNodes = 0;
                return;
            end

            [path, solInfo, treeA, treeB, numIters] = ...
                plan@matlabshared.planning.internal.BiRRT(obj, startState, goalState);

            if solInfo.IsPathFound
                pathObj = navPath(obj.StateSpace, path);
            else
                pathObj = navPath(obj.StateSpace);
            end

            solnInfo                    = struct();
            solnInfo.IsPathFound        = solInfo.IsPathFound;
            solnInfo.ExitFlag           = solInfo.ExitFlag;
            solnInfo.NumIterations      = numIters;
            solnInfo.StartTreeData      = treeA.inspect()';            
            
            % Remove NaN's dummy data which has been stored at starting in
            % GoalTree in BiRRT.
            solnInfo.GoalTreeData       = treeB.inspect()';
            % Number of nodes in the start tree and goal tree are defined
            % excluding the root node. Hence subtracting 1 from the total  
            % tree nodes. 
            solnInfo.StartTreeNumNodes  = treeA.getNumNodes()-1;
            solnInfo.GoalTreeNumNodes   = treeB.getNumNodes()-1;
        end

        function newObj = copy(obj)
            %copy Create deep copy of planner object
            %   PLANNER2 = copy(PLANNER1) creates a deep copy of the 
            %   specified planner object.

            validator = obj.StateValidator.copy;

            newObj = plannerBiRRT(validator.StateSpace, validator);
            newObj.MaxIterations = obj.MaxIterations;
            newObj.MaxNumTreeNodes = obj.MaxNumTreeNodes;
            newObj.MaxConnectionDistance = obj.MaxConnectionDistance;
            newObj.EnableConnectHeuristic = obj.EnableConnectHeuristic;
        end

    end

    % Setters
    methods

        function set.EnableConnectHeuristic(obj, enableConnectHeuristic)
            %set.EnableConnectHeuristic Setter of EnableConnectHeuristic
            validateattributes(enableConnectHeuristic, {'logical'},...
                {'nonempty', 'scalar'},...
                getClassName(obj), ...
                'EnableConnectHeuristic');
            obj.EnableConnectHeuristic = enableConnectHeuristic;
        end

        function set.MaxNumTreeNodes(obj, maxNumNodes)
            %set.MaxNumTreeNodes
            robotics.internal.validation.validatePositiveIntegerScalar(maxNumNodes, getClassName(obj), 'MaxNumTreeNodes');
            obj.MaxNumTreeNodesInternal = maxNumNodes;
        end

        function set.MaxIterations(obj, maxIter)
            %set.MaxIterations
            robotics.internal.validation.validatePositiveIntegerScalar(maxIter, getClassName(obj), 'MaxIterations');
            obj.MaxIterations = maxIter;
        end

        function set.MaxConnectionDistance(obj, maxConnDist)
            %set.MaxConnectionDistance
            robotics.internal.validation.validatePositiveNumericScalar(maxConnDist, getClassName(obj), 'MaxConnectionDistance');
            obj.MaxConnectionDistance = maxConnDist;
        end
    end

    %Getters for Dependent properties
    methods

        function maxNumNodes = get.MaxNumTreeNodes(obj)
            %get.MaxNumTreeNodes Getter for MaxNumTreeNodes
            maxNumNodes = obj.MaxNumTreeNodesInternal;
        end

        function stateValidator = get.StateValidator(obj)
            %get.StateValidator Getter for StateValidator
            stateValidator = obj.StateValidatorInternal;
        end

        function stateSpace = get.StateSpace(obj)
            %get.StateSpace Getter for StateSpace
            stateSpace = obj.StateValidatorInternal.StateSpace;
        end
    end

    methods(Access=protected)
        function cname = getClassName(obj) %#ok<MANU>
            %getClassName Returns the name of the class
            cname = "plannerBiRRT";
        end
                
        function state = sampleFromGoalRegionAndProject(obj)
        %sampleFromGoalRegionAndProject Concrete template method of BiRRT

            state = zeros(1, obj.StateSpace.NumStateVariables);
        end
    end

    methods(Access=private)
        function validateConstructorInput(obj, ss, sv)
            %validateConstructorInput

            validateattributes(ss, {'nav.StateSpace'}, {'scalar', 'nonempty'}, 'plannerBiRRT', 'stateSpace');
            validateattributes(sv, {'nav.StateValidator'}, {'scalar', 'nonempty'}, 'plannerBiRRT', 'stateValidator');

            if coder.target('MATLAB')
                if ss == sv.StateSpace % reference to the same state space object
                    obj.StateSpace = ss;
                    obj.StateValidator = sv;
                else
                    coder.internal.error('nav:navalgs:plannerrrt:RequireSameStateSpace');
                end
            else
                % ignore the handle check during codegen
                obj.StateSpace = ss;
                obj.StateValidator = sv;
            end
        end
    end
end