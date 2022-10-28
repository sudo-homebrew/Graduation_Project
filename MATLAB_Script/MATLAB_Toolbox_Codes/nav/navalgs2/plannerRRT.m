classdef plannerRRT < nav.algs.internal.InternalAccess
    %PLANNERRRT Create geometric rapidly-exploring random tree path planner 
    %   plannerRRT creates a rapidly-exploring random tree (RRT) planner 
    %   for solving geometric planning problems. RRT is a tree-based motion
    %   planner that builds a search tree incrementally from samples 
    %   randomly drawn from a given state space. The tree will eventually 
    %   span the search space and connects the start state and the goal
    %   state. The general tree growing process is as follows:
    %
    %   RRT samples a random state, STATERAND, in the state space, then it 
    %   finds a state, STATENEAREST, that is already in the search tree and
    %   is closest to STATERAND (closest as defined in the state space). 
    %   The algorithm then expands from STATENEAREST towards STATERAND, 
    %   until a state, STATENEW, is reached. Then STATENEW is added to the 
    %   search tree. 
    %
    %   For geometric RRT, the expansion/connection between two states can  
    %   always be found analytically without violating the constrains 
    %   specified in the state space of the planner. 
    %
    %   plannerRRT(STATESPACE, STATEVAL) creates an RRT planner from a 
    %   state space object, STATESPACE, and a state validator object,
    %   STATEVAL. STATEVAL's state space must be the same as STATESPACE.
    %
    %   plannerRRT properties:
    %      StateSpace            - State space for the planner
    %      StateValidator        - State validator for the planner
    %      MaxNumTreeNodes       - Max number of tree nodes
    %      MaxIterations         - Max number of iterations  
    %      MaxConnectionDistance - Max connection distance between two states
    %      GoalReachedFcn        - Callback for determining if goal is reached
    %      GoalBias              - Probability of choosing goal state during state sampling
    %
    %   plannerRRT methods:
    %      plan    - Plan a path between two states
    %      copy    - Create deep copy of the planner object
    %
    %   Example:
    %      % Create a state space
    %      ss = stateSpaceSE2;
    %
    %      % Create a occupancyMap-based state validator using the state space just created
    %      sv = validatorOccupancyMap(ss);
    %
    %      % Update the map for state validator and set map resolution as 10 cells/meter
    %      load exampleMaps
    %      map = occupancyMap(simpleMap, 10);
    %      sv.Map = map;
    %
    %      % Set validation distance for the validator
    %      sv.ValidationDistance = 0.01;
    %
    %      % Update state space bounds to be the same as map limits
    %      ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    %
    %      % Create the path planner and increase max connection distance
    %      planner = plannerRRT(ss, sv);
    %      planner.MaxConnectionDistance = 0.3;
    %
    %      % Set the start and goal states
    %      start = [0.5, 0.5 0];
    %      goal = [2.5, 0.2, 0];
    %
    %      % Plan a path with default settings, set rng for repetitive result
    %      rng(100, 'twister')
    %      [pthObj, solnInfo] = plan(planner, start, goal)
    %
    %      % Show the results
    %      map.show; hold on;
    %      plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-'); % tree expansion
    %      plot(pthObj.States(:,1), pthObj.States(:,2), 'r-', 'LineWidth', 2) % draw path
    %     
    %
    %   References: 
    %   [1] S.M. Lavalle, J.J. Kuffner, "Randomized kinodynamic planning",
    %       International Journal of Robotics Research, vol. 20, no. 5,
    %       pp. 378-400, May 2001 
    %
    %   See also plannerRRTStar, plannerBiRRT.
    
    %   Copyright 2019-2021 The MathWorks, Inc.
    
    %#codegen
    
    properties (Constant, Access=protected)
        GoalReached = 1
        MaxIterationsReached = 2
        MaxNumTreeNodesReached = 3
        MotionInCollision = 4
        InProgress = 5
        GoalReachedButContinue = 6
        ExistingState = 7
        Unassigned = 8
        
    end    
    
    properties (SetAccess = {?nav.algs.internal.InternalAccess})
        %StateSpace State space for the planner
        StateSpace
        
        %StateValidator State validator for the planner
        StateValidator
    end
    
    properties
        
        %MaxNumTreeNodes Max number of nodes in the search tree
        %   This number does not count the root node.
        %
        %   Default: 1e4
        MaxNumTreeNodes
        
        %MaxIterations Max number of iterations 
        %   This is also the max number of calls to the "extend" routine
        %
        %   Default: 1e4
        MaxIterations

        %MaxConnectionDistance Maximum length of a motion to be added to tree
        %
        %   Default: 0.1
        MaxConnectionDistance
        
        %GoalReachedFcn Callback function that checks whether goal is reached
        %   The callback function should accept three input arguments.
        %   The first argument is the planner object. The second argument
        %   is the state to check. The third argument is the goal state.
        %   States are row vectors with length consistent with the
        %   dimension of the planner's state space. The output of the
        %   callback function should be a Boolean that indicates whether
        %   goal is reached or not.
        %
        %   The expected function signature is as follows:
        %
        %      function isReached = goalReachedFcn(PLANNER, CURRENTSTATE, GOALSTATE)
        %
        %   Default: @nav.algs.checkIfGoalIsReached
        %
        %   To use custom GoalReachedFcn in code generation workflow  
        %   this property must be set to a custom function handle before 
        %   calling the plan method and it cannot be changed afterwards.
        GoalReachedFcn
        
        %GoalBias Probability of choosing goal state during state sampling
        %    This property defines the probability of choosing the actual 
        %    goal state during the process of randomly selecting states
        %    from the state space. It is a real number between 0.0 and 1.0.
        %    The value should be around 0.05 and should not be too large.
        %    Using the default value is usually a good start.
        %
        %    Default: 0.05
        GoalBias
    end
    
    properties (Access = {?nav.algs.internal.InternalAccess})

        %KeepIntermediateStates
        KeepIntermediateStates
        
        %NearestNeighborMethod
        NearestNeighborMethod
        
        %MaxTime
        MaxTime
        
        %RandomSamplePool Cache for pre-generated random state samples
        RandomSamplePool
        
        %PregenerateSamples Whether to pre-generate samples
        PregenerateSamples
    end
    
    properties (Access = protected)
        
        CurrentGoalState
    end

    methods
        function obj = plannerRRT(stateSpace, stateValidator)
            %PLANNERRRT Constructor
            obj.validateConstructorInput(stateSpace, stateValidator);
            obj.StateSpace = stateSpace;
            obj.StateValidator = stateValidator;
            obj.MaxNumTreeNodes = 1e4;
            obj.MaxIterations = 1e4;
            obj.MaxConnectionDistance = 0.1;
            obj.GoalBias = 0.05;
            % codegen does not allow redefinition of function handles once
            % initialized. Hence for codegen workflow initializing
            % GoalReachedFcn in 'plan' function if the user does not
            % provide custom GoalReachedFcn.
            if coder.target('MATLAB')
                obj.GoalReachedFcn = @nav.algs.checkIfGoalIsReached;
            end
        end
        
        function [pathObj, solnInfo] = plan(obj, startState, goalState)
            %plan Plan a path between two states
            %   PATH = plan(PLANNER, STARTSTATE, GOALSTATE) tries to find  
            %   a valid path between STARTSTATE and GOALSTATE. The planning
            %   is carried out based on the underlying state space and state
            %   validator of PLANNER. The output, PATH, is returned as a 
            %   navpath object.
            %
            %   [PATH, SOLNINFO] = plan(PLANNER, ...) also returns a struct,
            %   SOLNINFO, as a second output that gives additional    
            %   details regarding the planning solution.
            %   SOLNINFO has the following fields:
            %
            %   IsPathFound:  Boolean indicating whether a path is found
            %
            %      ExitFlag:  A number indicating why planner terminates
            %                 1 - 'GoalReached'
            %                 2 - 'MaxIterationsReached'
            %                 3 - 'MaxNumNodesReached'
            %
            %      NumNodes:  Number of nodes in the search tree when
            %                 planner terminates (not counting the root
            %                 node).
            %
            % NumIterations:  Number of "extend" routines executed
            %
            %      TreeData:  A collection of explored states that reflects
            %                 the status of the search tree when planner
            %                 terminates. Note that nan values are inserted
            %                 as delimiters to separate each individual
            %                 edge.

            if coder.target('MATLAB')
                cleaner = onCleanup(@()obj.cleanUp);
            end
            %validate the startState and goalState
            [startState, goalState] = validateStartGoal( ...
                obj, startState, goalState);

            % For codegen workflow, initialize GoalReachedFcn to default if
            % the user does not provide custom GoalReachedFcn. is_defined()
            % return true if the coder has seen a type for the property before.
            if ~coder.internal.is_defined(obj.GoalReachedFcn)
                obj.GoalReachedFcn = @nav.algs.checkIfGoalIsReached;
            end

            if obj.GoalReachedFcn(obj, startState, goalState)
                pathObj = navPath(obj.StateSpace, [startState; goalState]);
                solnInfo = struct();
                solnInfo.IsPathFound = true;
                solnInfo.ExitFlag = obj.GoalReached;
                solnInfo.NumNodes = 1;
                solnInfo.NumIterations = 0;
                solnInfo.TreeData = [startState;...
                    nan(1,obj.StateSpace.NumStateVariables); ...
                    startState;...
                    goalState;...
                    nan(1,obj.StateSpace.NumStateVariables)];
                assignPathCostForIteration(obj, pathObj.pathLength(), 1);
                obj.cleanUp();
                return;
            end

            obj.CurrentGoalState = goalState;
            tentativeGoalIds = [];
            
            treeInternal = initializeSearchTree(obj, startState);
            %populate the random sample pool and skip state validation for
            % manipulatorStateSpace/CollisionBodyValidator
            setupPlanLoop(obj);
            
            obj.preLoopSetup(treeInternal);
            
            pathFound = false;
            statusCode = obj.Unassigned;
            numIterations = 0;
            for k = 1:obj.MaxIterations
                if obj.PregenerateSamples
                    randState = obj.RandomSamplePool(k,:); % dispense random sample from the pool
                else
                    randState = obj.StateSpace.sampleUniform();
                end

                if rand() < obj.GoalBias
                    randState = obj.CurrentGoalState;
                end

                [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal);

                if statusCode == obj.GoalReached 
                    pathFound = true;
                    tentativeGoalIds = [tentativeGoalIds, newNodeId]; %#ok<AGROW>
                    numIterations = k;
                    break;
                end

                if statusCode == obj.GoalReachedButContinue
                    pathFound = true;
                    tentativeGoalIds = [tentativeGoalIds, newNodeId]; %#ok<AGROW>
                end

                % Record the minCost in this iteration. For RRT there is
                % nothing to be done, see implementation in RRT* for
                % details.
                collectPathCosts(obj, treeInternal, tentativeGoalIds, k);

                if statusCode == obj.MaxNumTreeNodesReached
                    numIterations = k;
                    break;
                end
            end
            
            [pathObj, solnInfo]= postPlanningLoop( obj,...
                treeInternal, tentativeGoalIds, numIterations, statusCode, pathFound);

            obj.cleanUp();

        end

        function objCopied = copy(obj)
            %COPY Create a deep copy of the plannerRRT object
            %   PLANNERCOPIED = COPY(PLANNER) creates a deep copy of plannerRRT
            %   object, PLANNER, and returns the new object in PLANNERCOPIED.
            
            validator = obj.StateValidator.copy;
            
            objCopied = plannerRRT(validator.StateSpace, validator);
            obj.copyProperties(objCopied);
        end
    end
    
    
    methods (Access = {?nav.algs.internal.InternalAccess})
        
        function preLoopSetup(~, ~)
            %preLoopSetup
            
            % do nothing for plannerRRT
        end
        
        function [newNodeId, statusCode, treeInternal] = extend(obj, randState, treeInternal)
            %extend The RRT "Extend" routine
            statusCode = obj.InProgress;
            newNodeId = nan;
            idx = treeInternal.nearestNeighbor(randState);
            nnState = treeInternal.getNodeState(idx);
                
            d = obj.StateSpace.distance(randState, nnState);
            newState = randState;
            
            % steer
            if d > obj.MaxConnectionDistance
                newState = obj.StateSpace.interpolate(nnState, randState, obj.MaxConnectionDistance/d);  % L/d*(randState - nnState) + nnState;
            end

            % check motion validity
            if ~obj.StateValidator.isMotionValid(nnState, newState)
                statusCode = obj.MotionInCollision;
                return;
            end

            newNodeId = treeInternal.insertNode(newState, idx);
            if obj.GoalReachedFcn(obj, obj.CurrentGoalState, newState)
                statusCode = obj.GoalReached;
                return;
            end
            
            if newNodeId == obj.MaxNumTreeNodes
                statusCode = obj.MaxNumTreeNodesReached;
                return
            end
            
        end

        function cleanUp(obj)
            %cleanUp To clean up after plan
            svInternal = obj.StateValidator;
            ssInternal = svInternal.StateSpace;
            switch class(ssInternal)
              case 'stateSpaceSE2'
                ssInternal.SkipStateValidation = false;
              case 'stateSpaceDubins'
                ssInternal.SkipStateValidation = false;
              case 'stateSpaceReedsShepp'
                ssInternal.SkipStateValidation = false;
              case 'manipulatorStateSpace'
                ssInternal.SkipStateValidation = false;
            end
            if isa(obj.StateValidator, 'validatorOccupancyMap') || ...
                    isa(obj.StateValidator, 'manipulatorCollisionBodyValidator')
                obj.StateValidator.SkipStateValidation = false;
                obj.StateSpace.SkipStateValidation = false;
            end
        end

        function treeInternal = initializeSearchTree(obj, startState)
            %initializeSearchTree create a searchTree with the specified startState
            % and MaxNumTreeNodes 
            %   TREE = initializeSearchTree(OBJ, STARTSTATE) returns an
            %   internal SearchTree object with specified STARTSTATE as
            %   well as MAXNUMTREENODES. The TREE object is configured
            %   based on the properties of the specified OBJ.STATESPACE 
            treeInternal = nav.algs.internal.SearchTree(startState, obj.MaxNumTreeNodes);
            extendsReversely = true;
            switch class(obj.StateSpace)
                case 'stateSpaceSE2'
                    weights = [obj.StateSpace.WeightXY, obj.StateSpace.WeightXY, obj.StateSpace.WeightTheta];
                    topologies = [0 0 1];
                    treeInternal.configureCommonCSMetric(topologies, weights, ~extendsReversely);
                    obj.StateSpace.SkipStateValidation = true;
                    obj.PregenerateSamples = true;
                case 'stateSpaceDubins'
                    treeInternal.configureDubinsMetric(obj.StateSpace.MinTurningRadius, ~extendsReversely);
                    obj.StateSpace.SkipStateValidation = true;
                    obj.PregenerateSamples = true;
                case 'stateSpaceReedsShepp'
                    treeInternal.configureReedsSheppMetric(obj.StateSpace.MinTurningRadius, obj.StateSpace.ReverseCost, ~extendsReversely);
                    obj.StateSpace.SkipStateValidation = true;
                    obj.PregenerateSamples = true;
                case 'manipulatorStateSpace'
                    obj.StateSpace.SkipStateValidation = true;
                    obj.PregenerateSamples = true;
                otherwise
                    ss = obj.StateSpace;
                    treeInternal.setCustomizedStateSpace(ss, ~extendsReversely);
            end

        end

        function [pathObj, solnInfo] = postPlanningLoop( obj,...
                treeInternal, tentativeGoalIds, numIterations, statusCode, pathFound)
            %postPlanningLoop derive the pathObj and solnInfo from the status flags
            %   PATHOBJ = postPlanningLoop(OBJ, TREEINTERNAL, TENTATIVEGOALIDS,
            %   NUMITERATIONS, STATUSCODE, PATHFOUND) returns a NAVPATH object
            %   from OBJ.PATHSTATES and OBJ.STATESPACE which contains the path found.  
            %   [PATHOBJ, SOLNINFO] = postPlanningLoop(OBJ,...) returns a
            %   structure SOLNINFO with additional path information.  

            if numIterations == 0
                numIterations = obj.MaxIterations;
            end

            treeData = treeInternal.inspect();
            numNodes = treeInternal.getNumNodes()-1;

            exitFlag = statusCode;
            if statusCode >= obj.MotionInCollision
                exitFlag = obj.MaxIterationsReached;
            end

            if pathFound
                costBest = inf;
                idBest = -1;
                for j = 1:length(tentativeGoalIds)
                    nid = tentativeGoalIds(j);
                    c = treeInternal.getNodeCostFromRoot(nid);
                    if c < costBest
                        idBest = nid;
                        costBest = c;
                    end
                end

                % Record the best cost for the last iteration, this is
                % needed in case last iteration was the first time path was
                % found.
                assignPathCostForIteration(obj, costBest, numIterations);

                pathStates = treeInternal.tracebackToRoot(idBest);

                pathObj = navPath(obj.StateSpace, flip(pathStates'));
            else
                pathObj = navPath(obj.StateSpace);
            end

            solnInfo = struct();
            solnInfo.IsPathFound = pathFound;
            solnInfo.ExitFlag = exitFlag;
            solnInfo.NumNodes = numNodes;
            solnInfo.NumIterations = numIterations;
            solnInfo.TreeData = treeData';
        end

        function [startState, goalState] = validateStartGoal( ...
                obj, startState, goalState)
            %validateStartGoal check whether the startState and goalState
            % are valid
            if ~all(obj.StateValidator.isStateValid(startState)) || ...
                    ~all(all(isfinite(startState)))
                coder.internal.error('nav:navalgs:plannerrrt:StartStateNotValid');
            end

            if ~all(obj.StateValidator.isStateValid(goalState)) || ...
                    ~all(all(isfinite(goalState)))
                coder.internal.error('nav:navalgs:plannerrrt:GoalStateNotValid');
            end

            startState = nav.internal.validation.validateStateVector(startState, ...
                obj.StateSpace.NumStateVariables, 'plan', 'startState');
            goalState = nav.internal.validation.validateStateVector(goalState, ...
                obj.StateSpace.NumStateVariables, 'plan', 'goalState');

        end

        function setupPlanLoop(obj)
            %setupPlanLoop populate the random sample pool and skip state 
            % validation for validatorOccupancyMap and CollisionBodyValidator
            
            s = rng;
            if obj.PregenerateSamples
                %populate random sample pool
                obj.RandomSamplePool = obj.StateSpace.sampleUniform(obj.MaxIterations);
            else
                obj.RandomSamplePool = zeros(1,obj.StateSpace.NumStateVariables);
            end
            rng(s); % should not let the maxIterations affect the goalBias result

            if isa(obj.StateValidator, 'manipulatorCollisionBodyValidator')
                obj.StateValidator.SkipStateValidation = true;
            end

            if isa(obj.StateValidator, 'validatorOccupancyMap')
                obj.StateValidator.SkipStateValidation = true;
                obj.StateValidator.configureValidatorForFastOccupancyCheck();
            end
        end
    end
    
    
    methods (Access = protected)
        function validateConstructorInput(obj, ss, sv)
            %validateConstructorInput
            
            validateattributes(ss, {'nav.StateSpace'}, {'scalar', 'nonempty'}, 'plannerRRT', 'stateSpace');
            validateattributes(sv, {'nav.StateValidator'}, {'scalar', 'nonempty'}, 'plannerRRT', 'stateValidator');
            
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
        
        function cname = getClassName(obj) %#ok<MANU>
            %getClassName
            cname = 'plannerRRT';
        end
        
        function copyProperties(obj, copyObj)
            %copyProperties Copy property data from this object to a new object
            copyObj.MaxIterations = obj.MaxIterations;
            copyObj.MaxNumTreeNodes = obj.MaxNumTreeNodes;
            copyObj.MaxConnectionDistance = obj.MaxConnectionDistance;
            %For codegen workflow, the default GoalReachedFcn is only assigned 
            % in the plan function. Hence copy the property only if the user 
            % has provided a custom GoalReachedFcn.
            if coder.internal.is_defined(obj.GoalReachedFcn)
                copyObj.GoalReachedFcn = obj.GoalReachedFcn;
            end
            copyObj.GoalBias = obj.GoalBias;
        end

        function collectPathCosts(~, ~, ~, ~)
            %collectPathCosts calculates min cost in tree for IDs and save it.
            % This is a stub and do nothing for plannerRRT
        end

        function assignPathCostForIteration(~, ~, ~)
            %assignPathCostForIteration assigns cost to the element
            %corresponding to iterationNum in PathCostAtIteration property.            
            % do nothing for plannerRRT
        end
    end
    
    % setters
    methods
        function set.GoalReachedFcn(obj, goalReachedFcn)
            %set.GoalReachedFcn Set the handle to the function that
            %   determines if goal is reached
            validateattributes(goalReachedFcn, {'function_handle'}, {'nonempty'}, getClassName(obj), 'GoalReachedFcn');
            obj.GoalReachedFcn = goalReachedFcn;
        end
        
        function set.MaxNumTreeNodes(obj, maxNumNodes)
            %set.MaxNumTreeNodes
            robotics.internal.validation.validatePositiveIntegerScalar(maxNumNodes, getClassName(obj), 'MaxNumTreeNodes');
            obj.MaxNumTreeNodes = maxNumNodes;
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
        
        function set.GoalBias(obj, goalBias)
            %set.GoalBias
            validateattributes(goalBias, {'double'}, ...
                 {'nonempty', 'scalar', 'real', 'nonnan', 'finite', '>=', 0.0, '<=', 1.0}, ...
                 getClassName(obj), 'GoalBias');
            obj.GoalBias = goalBias;
        end
        
    end
end

