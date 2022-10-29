classdef plannerControlRRT < nav.algs.internal.InternalAccess
%plannerControlRRT Create control-based rapidly-exploring random tree
%planner
%
%   plannerControlRRT creates a rapidly-exploring random tree (RRT) planner
%   for solving using kinematic or dynamic planning problems. RRT is a
%   tree-based motion planning routine that incrementally grows a search
%   tree. In kinematic planners, the tree is grown by randomly sampling
%   states in system's configuration space, and then attempting to
%   propagate the nearest node towards that state using a given system's
%   kinematic model and control policies. Propagation can be directed or
%   undirected depending on the policies implemented by the
%   StatePropagator. The tree will eventually span the search space and
%   connect the start and goal states. The typical search process is as
%   follows:
%
%   RRT samples a random state in the state space, qTgt, and finds an
%   existing state in the tree, qNear, which offers the least-costly
%   propagation towards qTgt. The StatePropagator then generates a control
%   command and duration, uInit and numStep, and propagates the system
%   towards the target. If propagation was successful, then the planner
%   adds the final state, command, elapsed number of steps and target to
%   the tree. Once a node has been added, the planner can either attempt to
%   propagate qNew towards the goal-state a specified number of times, or
%   begin the next iteration.
%   
%   There are a few important difference between geometric and kinematic
%   RRT:
%
%   The benefit to a kinodynamic planner is that it is guaranteed to return
%   a sequence of states, controls, and references which comprise a
%   kinematically or dynamically feasible path.
%
%   The drawback to this planner is that kinematic propagations cannot
%   guarantee that qNew is exactly equal to qTgt unless there exists an
%   analytic representation for a sequence of controls that drive the
%   system between two configurations with zero residual error.
%
%   In general, this means that kinodynamic planners are typically
%   asymptotically complete and guarantee kinematic feasibility, but often
%   cannot guarantee asymptotic optimality.
%
%   plannerObj = plannerControlRRT(STATEPROPAGATOR) creates a kinodynamic
%   RRT planner from a state propagator object, STATEPROPAGATOR.
%
%   plannerControlRRT properties:
%      StatePropagator          - Represents the problem-specific planning space
%      ContinueAfterGoalReached - Continue to search for additional paths after goal is reached
%      MaxPlanningTime          - Maximum time allowed for planning
%      MaxNumTreeNode           - Maximum number of nodes allowed in tree
%      MaxNumIteration          - Maximum number of iterations
%      NumGoalExtension         - Number times to propagateWhileValid towards goal
%      GoalBias                 - Probability of choosing goal state during state sampling
%      GoalReachedFcn           - Callback for determining if goal is reached
%
%   plannerControlRRT methods:
%      plan    - Plan a kinematically feasible path between two states
%      copy    - Create deep copy of the planner object
%
%   Example:
%       % Create a state propagator which uses a steer-angle bike model
%       bikePropagator = mobileRobotPropagator;
%       
%       % Create a planner for the bicycle propagation problem
%       planner = plannerControlRRT(bikePropagator);
%       
%       % Set the start and goal configurations
%       start = [0 0 0];
%       goal = [10 20 pi/3];
%       
%       % Plan a path between two states in the empty environment
%       pathObj = plan(planner,start,goal);
%
%   References: 
%   [1] S.M. Lavalle, J.J. Kuffner, "Randomized kinodynamic planning",
%       International Journal of Robotics Research, vol. 20, no. 5,
%       pp. 378-400, May 2001 
%   
%   [2] Kavraki, L. and S. LaValle. "Chapter 5 Motion Planning" 1st ed.,
%       B. Siciliano et O. Khatib, Ed. New York: Springer-Verlag Berlin 
%       Heidelberg, 2008, pp. 109-131.
%
%   See also plannerRRT, plannerRRTStar, plannerBiRRT

%   Copyright 2021 The MathWorks, Inc.

    %#codegen
    
    properties
        %StatePropagator Represents the problem-specific planning space
        %
        %   The StatePropagator defines the configuration and control
        %   spaces. It provides implementations for state and control
        %   sampling, system propagation, and provides cost, distance, and 
        %   goal-reached metrics specific to the planning problem.
        StatePropagator
        
        %ContinueAfterGoalReached Continue to search for additional paths after goal is reached
        %   Planning will stop after the max number of iterations, max
        %   number of tree nodes, or max planning time has been is reached.
        %
        %   Default: false
        ContinueAfterGoalReached = false;
        
        %MaxPlanningTime Maximum time allowed for planning
        %
        %   Default: inf
        MaxPlanningTime = inf;
        
        %MaxNumTreeNode Maximum number of nodes allowed in tree
        %
        %   Default: 1e4
        MaxNumTreeNode = 1e4;
        
        %MaxNumIteration Maximum number of iterations
        %
        %   Default: 1e4
        MaxNumIteration = 1e4;
        
        %NumGoalExtension Number times to propagate towards goal
        %
        %   After a new node has successfully been added to the tree, the
        %   planner will attempt to propagate the new node towards the
        %   goal using propagateWhileValid. It will continue propagating
        %   until propagateWhileValid returns an empty state vector, the
        %   goal is reached, or it has been called NumGoalExtension times.
        %
        %   This behavior can be turned off by setting this property to 0
        %
        %   Default: 1
        NumGoalExtension = 1;
        
        %GoalBias Probability of choosing goal state during state sampling
        %
        %   Default: 0.05
        GoalBias = 0.05;
        
        %GoalReachedFcn Callback for determining if goal is reached
        %
        %   [tf, err] = goalReached(planner, q, qTgt) Determines whether
        %   the current state, q (1xN), has reached the target state, 
        %   qTgt (1xN). 
        %
        %   The function should a boolean, tf, and residual error, err, 
        %   between q, qTgt.
        %   
        %   [TF, ERR] = goalReached(planner, Q, qTgt) Takes an M-row matrix
        %   of states, Q, and returns an Mx1 vector of booleans, TF, and
        %   Mx1 vector of residual errors, ERR.
        %
        %   Default: @plannerControlRRT.GoalReachedDefault
        GoalReachedFcn
    end
    
    properties (Hidden)
        %PlotUpdateInterval
        PlotUpdateInterval = inf;
    end
    
    properties (Access = protected)
        %MinDistToGoal A vector that stores the minimum distance to goal during planning
        MinDistToGoal
    end
    
    properties (Access = ?nav.algs.internal.InternalAccess)
        %PlanCallback A callback function executed during planning
        PlanCallback
        
        %ExitCallback A callback function executed prior to plan exit
        ExitCallback
        
        %NumGoalFound Number of connections made to goal region
        NumGoalFound = 0;
        
        %NumIter Number of iterations
        NumIter = 0;
        
        %NumNode Number of nodes added to tree
        NumNode = 0;
    end
    
    methods
        function obj = plannerControlRRT(sp,varargin)
            narginchk(1,15);
            nvPairs = plannerControlRRT.parseInputs(varargin{:});
            obj.StatePropagator = sp;
            names = fieldnames(nvPairs);
            for i = 1:numel(names)
                obj.(names{i}) = nvPairs.(names{i});
            end
             obj.PlanCallback = @plannerControlRRT.PlanCallbackDefault;
             obj.ExitCallback = @plannerControlRRT.ExitCallbackDefault;
        end
        
        function [path, solnInfo] = plan(obj, start, goal)
        %plan Plan a kinematically feasible path between two states
        %
        %   PATH = plan(OBJ, STARTSTATE, GOALSTATE) tries to find a valid
        %   path between STARTSTATE and GOALSTATE. The planning is carried
        %   out based on the state propagator, which leverages a kinematic 
        %   model and controllers of the system to search the configuration
        %   space. The planner returns a navControlPath object, PATH, which
        %   contains the propagator used during planning and a sequence of 
        %   states, controls, target states, and control durations.
        %
        %   [___, SOLNINFO] = plan(OBJ, STARTSTATE, GOALSTATE) returns
        %   the optional SOLNINFO struct, which provides additional details
        %   regarding the planning solution. 
        %
        %       SOLNINFO has the following fields:
        % 
        %       IsPathFound:  Boolean indicating whether a path is found
        % 
        %          ExitFlag:  A number indicating why planner terminates
        %                     1 - 'GoalReached'
        %                     2 - 'MaxIterationsReached'
        %                     3 - 'MaxNumNodesReached'
        %                     4 - 'MaxPlanningTimeExceeded'
        % 
        %       NumTreeNode:  Number of nodes in the search tree when
        %                     planner terminates (not counting the root
        %                     node).
        % 
        %      NumIteration:  Number of target states propagated
        %
        %      PlanningTime:  Time elapsed during call to plan
        % 
        %          TreeData:  A collection of explored states that reflects
        %                     the status of the search tree when planner
        %                     terminates. Note that nan values are inserted
        %                     as delimiters to separate each individual
        %                     edge.
        %
        %   [___] = plan(OBJ, STARTSTATE, GOALSAMPLEFCN) takes in a
        %   function handle that produces a goal configuration when called.
        %   The function handle should take no inputs and generate a goal 
        %   state whose size matches STARTSTATE.
        
            % Start timer
            tic;
            
            % Validate inputs and initialize planner
            [sp, ss, tree, idDataMap, goalNodes, generateGoal] = obj.initializePlanner(start, goal);
            
            % Inputs passed by planner should always be valid, so input 
            % validation is turned off in NAV-owned features to save on 
            % computation. 
            if coder.target('MATLAB')
                % Create cleanup handle to re-activate input validation on 
                % shipped classes after planning completes. 
                cleanHandle = onCleanup(@()setValidationState(obj,true));
            end
            
            % Turn off validation during planning
            obj.setValidationState(false);
            
            pathFound = false;
            time = toc;
            
            % Define meta info indices
            u0Idx = 1:sp.NumControlOutput;
            qTIdx = sp.NumControlOutput+(1:ss.NumStateVariables);
            uFIdx = qTIdx(end)+u0Idx;
            
            % Create exit function
            exitFcn = @(i,n,dt,numGoalNodes) ...
                (i >= obj.MaxNumIteration) || ...
                (n >= obj.MaxNumTreeNode)  || ...
                (dt > obj.MaxPlanningTime) || ...
                (numGoalNodes > 0 && ~obj.ContinueAfterGoalReached);

            % Find path
            while ~exitFcn(obj.NumIter,obj.NumNode,time,obj.NumGoalFound)
                goalReached = false;
                
                if ~isinf(obj.PlotUpdateInterval) && mod(obj.NumIter,obj.PlotUpdateInterval) == 0
                    obj.PlanCallback(obj, tree, start, generateGoal());
                end
                
                if rand() < obj.GoalBias
                    % Draw a sample from the goal region
                    tgtState = generateGoal();
                else
                    % Draw a sample from the configuration space
                    tgtState = ss.sampleUniform(1);
                end
                
                % Find nearest neighbor based on approximate distance
                nnIdx = tree.nearestNeighbor(tgtState);
                curState = tree.getNodeState(nnIdx);
                curMeta  = idDataMap.getData(nnIdx);
                
                % Sample an initial control input for current conditions
                [uInit, maxNumStep] = sp.sampleControl(curState, curMeta(uFIdx), tgtState);
                
                if maxNumStep == 0
                    continue;
                end
                
                % Generate new node
                [iState, iCommand, numStep] = sp.propagateWhileValid(curState, uInit, tgtState, maxNumStep);
                
                if ~isempty(iState)
                    % Check if any of the states reached the goal
                    goalTarget = generateGoal();
                    sIdx = find(obj.GoalReachedFcn(obj, iState, goalTarget) == 1,1);
                    obj.MinDistToGoal(obj.NumNode+1) = min(obj.MinDistToGoal(obj.NumNode),min(sp.distance(iState,generateGoal())));
                    if ~isempty(sIdx)
                        % Goal reached
                        curState = iState(sIdx(1),:);
                        curMeta = [uInit tgtState iCommand(sIdx(1),:) nnIdx sum(numStep(1:sIdx(1)))+[0 curMeta(end)]];
                        goalReached = true;
                    else
                        % Goal not reached, add newest node to tree
                        curState = iState(end,:);
                        newIdx = tree.insertNode(iState(end,:),nnIdx);
                        curMeta = [uInit tgtState iCommand(end,:) nnIdx sum(numStep)+[0 curMeta(end)]];
                        idDataMap.insertData(newIdx,curMeta);
                        obj.NumNode = obj.NumNode+1;
                        
                        if obj.NumGoalExtension ~= 0
                            % Attempt goal extension
                            % Generate initial control input
                            tgtState = generateGoal();

                            % Sample an initial control input for current conditions
                            [uInit, maxNumStep] = sp.sampleControl(curState, curMeta(uFIdx), tgtState);

                            if maxNumStep == 0
                                continue;
                            end

                            for gExt = 1:obj.NumGoalExtension
                                % propagateWhileValid towards goal
                                [iState, iCommand, numStep] = sp.propagateWhileValid(curState, uInit, tgtState, maxNumStep);

                                % If goal was reached, add it to the list of valid goal nodes
                                if ~isempty(iState)
                                    curState = iState(end,:);
                                    curMeta = [uInit tgtState iCommand(end,:) newIdx sum(numStep)+[0 curMeta(end)]];
                                    
                                    if obj.GoalReachedFcn(obj,iState(end,:),tgtState)
                                        % Calculate cost of final segment
                                        goalReached = true;
                                        break;
                                    else
                                        newIdx = tree.insertNode(curState,newIdx);
                                        idDataMap.insertData(newIdx,curMeta);
                                        obj.NumNode = obj.NumNode+1;
                                        uInit = curMeta(uFIdx);
                                    end
                                    obj.MinDistToGoal(obj.NumNode) = min(obj.MinDistToGoal(obj.NumNode-1),min(sp.distance(iState,generateGoal())));
                                else
                                    break;
                                end
                            end
                        end
                    end
                    
                    if goalReached
                        pathFound = true;
                        obj.NumNode = obj.NumNode+1;
                        % Append goal segment to current set of goal segments
                        obj.NumGoalFound = obj.NumGoalFound+1;
                        goalNodes(obj.NumGoalFound) = struct(...
                            'PathCost', curMeta(end), ...
                            'GoalState', curState, ...
                            'GoalMeta', curMeta);
                    end
                end
                obj.NumIter = obj.NumIter+1;
                time = toc;
            end
            
            % Generate outputs
            [path, solnInfo] = obj.generateOutputs(tree, idDataMap, goalNodes, pathFound, time);
            
            if ~isinf(obj.PlotUpdateInterval)
                obj.ExitCallback(obj, tree, path, generateGoal);
            end
            
            % Turn validation on after planning completes
            obj.setValidationState(true);
        end
        
        function cObj = copy(obj)
        %copy Creates a deep copy of the planner
            spCpy = copy(obj.StatePropagator);
            vIn = {};
            names = fieldnames(obj.propertyDefaults());
            for i = 1:numel(names)
                vIn = {vIn{:} names{i} obj.(names{i})};
            end
            cObj = plannerControlRRT(spCpy,vIn{:});
        end
    end
    
    methods
        function set.StatePropagator(obj,sp)
            validateattributes(sp,{'nav.StatePropagator'},{'scalar'},'plannerControlRRT','StatePropagator',1);
            obj.StatePropagator = sp;
        end
        
        function set.ContinueAfterGoalReached(obj,continueFlag)
            validateattributes(continueFlag,{'numeric','logical'},{'binary','scalar'},'plannerControlRRT','ContinueAfterGoalReached',1);
            obj.ContinueAfterGoalReached = continueFlag;
        end
        
        function set.MaxPlanningTime(obj,maxTime)
            validateattributes(maxTime,{'numeric'},{'positive','scalar','nonnan'},'plannerControlRRT','MaxPlanningTime',1);
            obj.MaxPlanningTime = maxTime;
        end
        
        function set.MaxNumTreeNode(obj,maxNode)
            validateattributes(maxNode,{'numeric'},{'finite','positive','scalar'},'plannerControlRRT','MaxNumTreeNode',1);
            obj.MaxNumTreeNode = maxNode;
        end
        
        function set.MaxNumIteration(obj,maxIter)
            validateattributes(maxIter,{'numeric'},{'finite','positive','scalar'},'plannerControlRRT','MaxNumIteration',1);
            obj.MaxNumIteration = maxIter;
        end
        
        function set.NumGoalExtension(obj,goalExtension)
            validateattributes(goalExtension,{'numeric'},{'finite','nonnegative','scalar'},'plannerControlRRT','NumGoalExtension',1);
            obj.NumGoalExtension = goalExtension;
        end
        
        function set.GoalBias(obj,goalBias)
            validateattributes(goalBias,{'numeric'},{'nonnegative','scalar','<=',1,'nonnan'},'plannerControlRRT','GoalBias',1);
            obj.GoalBias = goalBias;
        end
        
        function set.GoalReachedFcn(obj,goalFcn)
            validateattributes(goalFcn,{'function_handle'},{'nonempty'},'plannerControlRRT','GoalReachedFcn',1);
            obj.GoalReachedFcn = goalFcn;
        end
    end
    
    methods (Access = ?nav.algs.internal.InternalAccess)
        function [sp, ss, tree, idDataMap, goalNodes, generateGoal] = initializePlanner(obj, start, goal)
        %initializePlanner Validate inputs and configure planner
            
            % Configure state propagator
            ss = obj.StatePropagator.StateSpace;
            sp = obj.StatePropagator;
            sp.setup();
            
            % Infer node size
            qSz = ss.NumStateVariables;
            uSz = sp.NumControlOutput;
            
            % Validate start and goal inputs
            validateattributes(start,{'numeric'},{'row','numel',qSz},'plan','start');
            if isnumeric(goal)
                validateattributes(start,{'numeric'},{'row','numel',qSz},'plan','goal');
                generateGoal = @()goal;
            else
                validateattributes(goal,{'function_handle'},{'scalar'},'plan','goal');
                generateGoal = goal;
            end
            
            % Initialize planner properties
            obj.NumIter = 0;
            obj.NumNode = 1;
            obj.NumGoalFound = 0;
            obj.MinDistToGoal = inf(obj.MaxNumTreeNode+1,1);
            obj.MinDistToGoal(1) = sp.distance(start,generateGoal());
            
            % Create search structures
            %   Search data is separated into three data structures
            %       tree: Internal searchTree containing states
            %           q: 1xQ state vector
            %       idDataMap: Container mapping nodeIDs to meta-info struct
            %           key: nodeID (numeric, scalar)
            %           value: idDataMap vector
            %               u0    :  initial control command (1xU)
            %               qTgt  :  target state (1xQ)
            %               uF    :  final control command (1xU)
            %               pID   :  parent node ID (numeric, scalar)
            %               nStep :  number of propagation steps (numeric, scalar)
            %               cost  :  total number of steps from root (numeric, scalar)
            %       goalNodes: Struct-array containing goal nodes
            %           PathCost  : cost
            %           GoalState : q
            %           GoalMeta  : idDataMap:value
            metaInfo = [zeros(1,uSz) nan(1,qSz) zeros(1,uSz) nan 0 0];
            tree = nav.algs.internal.SearchTree(start, obj.MaxNumTreeNode+1);
            tree.setCustomizedStateSpace(sp, false);
            idDataMap = nav.algs.internal.SimpleMap(numel(metaInfo));
            goalNodes = repmat(struct('PathCost',0,'GoalState',nan(1,qSz),'GoalMeta',metaInfo),obj.MaxNumTreeNode+1,1);
        end
        
        function [path, solnInfo] = generateOutputs(obj, tree, idDataMap, goalNodes, goalReached,time)
        %generateOutputs Construct path and solution info outputs
            sp = obj.StatePropagator;
            ss = sp.StateSpace;
            qSz = ss.NumStateVariables;
            uSz = sp.NumControlOutput;
            
            u0Idx = 1:sp.NumControlOutput;
            qTIdx = sp.NumControlOutput+(1:ss.NumStateVariables);
            
            if goalReached
                % Path found, determine best solution
                [~, bestGoalIdx] = min([goalNodes(1:obj.NumGoalFound,1).PathCost],[],2);
                bestState = goalNodes(bestGoalIdx).GoalState;
                bestMeta  = goalNodes(bestGoalIdx).GoalMeta;
                newIdx = tree.insertNode(bestState,bestMeta(end-2));
                idDataMap.insertData(newIdx,bestMeta);

                Q = flipud(tree.tracebackToRoot(newIdx)');
                nSeg = size(Q,1)-1;
                U = zeros(nSeg,uSz);
                qTgt = zeros(nSeg,qSz); 
                nSteps = zeros(nSeg,1);
                for i = nSeg:-1:1
                    U(i,:)      = bestMeta(u0Idx);
                    qTgt(i,:)   = bestMeta(qTIdx);
                    nSteps(i,:) = bestMeta(end-1);
                    bestMeta(:) = idDataMap.getData(bestMeta(end-2));
                end
                exitFlag = 1;
            else
                % No path found
                Q = zeros(0,qSz);
                U = zeros(0,uSz);
                qTgt = zeros(0,qSz);
                nSteps = zeros(0,1);
                
                if obj.NumIter >= obj.MaxNumIteration
                    % Exceeded maximum number of iterations 
                    exitFlag = 2;
                elseif obj.NumNode >= obj.MaxNumTreeNode
                    % Exceeded maximum number of nodes in tree
                    exitFlag = 3;
                else
                    % Exceeded maximum planning time
                    exitFlag = 4;
                end
            end

            % Return path object
            path = navPathControl(sp, Q, U, qTgt, nSteps*sp.ControlStepSize);
            
            % Return solution info
            solnInfo = struct(...
                'IsPathFound', goalReached, ...
                'ExitFlag', exitFlag, ...
                'NumTreeNode', obj.NumNode, ...
                'NumIteration', obj.NumIter, ...
                'PlanningTime', time, ...
                'TreeInfo',inspect(tree));
        end
    end
    
    methods (Access = protected)
        function setValidationState(obj,value)
        %setValidationState Activate or deactivate input validation on space and validator
            sp = obj.StatePropagator;
            
            if isa(sp,'mobileRobotPropagator')
                sp.SkipStateValidation = ~value;
                sv = sp.StateValidatorInternal;
                if isa(sv, 'validatorOccupancyMap')
                    sv.SkipStateValidation = ~value;
                    sv.StateSpace.SkipStateValidation = ~value;
                    sv.configureValidatorForFastOccupancyCheck();
                end
            end
        end
    end
    
    methods (Hidden, Static)
        function [TF, dist] = GoalReachedDefault(planner,q,qTgt)
            dist = planner.StatePropagator.distance(q,qTgt);
            TF = dist < 1;
        end
        
        function PlanCallbackDefault(planner, tree, start, goal) %#ok<INUSD>
        %PlanCallbackDefault No-op
        end
        
        function ExitCallbackDefault(planner, tree, path, goal) %#ok<INUSD>
        %PlanCallbackDefault No-op
        end
        
        function nvPairs = parseInputs(varargin)
            % Retrieve default properties
            defaults = plannerControlRRT.propertyDefaults;
            
            % Parse inputs and return NV-pairs
            pstruct = coder.internal.parseParameterInputs(defaults,struct(),varargin{:});
            nvPairs = coder.internal.vararginToStruct(pstruct,defaults,varargin{:});
        end
        
        function defaults = propertyDefaults()
            defaults = struct(...
            'ContinueAfterGoalReached',false,...
            'MaxPlanningTime',inf,...
            'MaxNumTreeNode',10000,...
            'MaxNumIteration',10000,...
            'NumGoalExtension',1,...
            'GoalBias',0.1,...
            'GoalReachedFcn',@plannerControlRRT.GoalReachedDefault);
        end
    end
end
