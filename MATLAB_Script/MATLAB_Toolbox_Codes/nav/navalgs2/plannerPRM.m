classdef plannerPRM < nav.algs.internal.InternalAccess
    %plannerPRM Create probabilistic roadmap path planner
    %   The probabilistic roadmap path planner constructs a roadmap without
    %   start or goal states. Use the plan method to find an obstacle-free 
    %   path between the specified start and goal states. If the plan method 
    %   does not find a connected path between the start and the goal states, 
    %   it returns an empty path.
    %
    %   planner = plannerPRM(STATESPACE, STATEVAL) creates a PRM planner from 
    %   a state space object, STATESPACE, and a state validator object, 
    %   STATEVAL. The state space of STATEVAL must be the same as STATESPACE.
    %   STATESPACE and STATEVAL also sets the StateSpace and StateValidator
    %   properties, respectively, of the planner.
    %
    %   planner = plannerPRM(...,Name=Value) sets properties using one or more 
    %   name-value pair arguments in addition to the input arguments in the 
    %   previous syntax. You can specify the MaxNumNodes or 
    %   MaxConnectionDistance properties as name-value pairs.
    %
    %   plannerPRM properties:
    %      StateSpace            - State space for the planner
    %      StateValidator        - State validator for the planner
    %      MaxNumNodes           - Maximum number of nodes in graph  
    %      MaxConnectionDistance - Maximum connection distance between two 
    %                              states
    %       
    %   plannerPRM methods:
    %       plan        - Plan path between two states
    %       copy        - Create deep copy of planner object
    %       graphData   - Retrieve graph as digraph object
    %
    %   Example:
    %       % Create a state space.
    %       ss = stateSpaceSE2;
    %
    %       % Create a state validator using the state space.
    %       sv = validatorOccupancyMap(ss);
    %
    %       % Update the map for state validator and set map.
    %       load exampleMaps;
    %       map = occupancyMap(simpleMap, 10);
    %       sv.Map = map;
    % 
    %       % Set validation distance for the validator.
    %       sv.ValidationDistance = 0.01;
    % 
    %       % Update state space bounds to be the same as map limits.
    %       ss.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
    % 
    %       % Create plannerPRM object, set rng for repetitive result.
    %       rng(100, "twister");
    %       planner = plannerPRM(ss, sv)
    % 
    %       % Retrieve graph as digraph object.
    %       digraphObj = planner.graphData
    %
    %       % Extract nodes and edges from graph.
    %       edges = table2array(digraphObj.Edges);
    %       nodes = table2array(digraphObj.Nodes);
    % 
    %       % Plot map and graph.
    %       show(sv.Map);
    %       hold on
    % 
    %       plot(nodes(:,1), nodes(:,2), "*", "Color", "b", "LineWidth", 2);
    % 
    %       for i = 1:size(edges,1)
    %           % Samples states at distance 0.02 meters.
    %           states = interpolate(ss, ...
    %               nodes(edges(i,1),:), nodes(edges(i,2),:),0:0.02:1);
    %           plot(states(:,1), states(:,2), "Color", "b");
    %       end
    % 
    %       % Set the start and goal states.
    %       start = [0.5, 0.5 0];
    %       goal = [2.5, 0.2, 0];
    % 
    %       plot(start(1), start(2), "*", "Color", "g", "LineWidth", 3);
    %       plot(goal(1), goal(2), "*", "Color", "r", "LineWidth", 3);
    % 
    %       % Plan a path with default settings.
    %       [pthObj, solnInfo] = plan(planner, start, goal)
    % 
    %       if solnInfo.IsPathFound
    %           interpolate(pthObj,1000);
    %           plot(pthObj.States(:,1), pthObj.States(:,2), "Color",...
    %              [0.85 0.325 0.098], "LineWidth",2);
    %       else
    %           disp("Path not found");
    %       end
    %       hold off
    %
    %   References:
    %
    %   [1] L.E. Kavraki, P. Svestka, J.C. Latombe, M.H. Overmars,
    %       "Probabilistic roadmaps for path planning in high-dimensional
    %       configuration spaces," IEEE Transactions on Robotics and
    %       Automation, vol. 12, no. 4, pp. 566-580, Aug 1996.
    %
    %   See also mobileRobotPRM, plannerRRT, plannerRRTStar, plannerBiRRT.
    
    %   Copyright 2021 The MathWorks, Inc.
    
    %#codegen
    
    properties (SetAccess = {?nav.algs.internal.InternalAccess})
        %StateSpace State space for the planner
        StateSpace

        %StateValidator State validator for the planner
        StateValidator
    end

    properties (SetAccess = private)
        %MaxConnectionDistance Maximum distance between two connected nodes
        %   Maximum distance between two connected nodes, specified as a 
        %   positive scalar in meters. Nodes with distance greater than the 
        %   MaxConnectionDistance will not be connected in the graph.
        %
        %   Default: inf
        MaxConnectionDistance;

        %MaxNumNodes Maximum number of nodes in graph
        %   Maximum number of nodes in the graph, specified as a positive 
        %   scalar. By increasing this value, the chance of finding a path 
        %   increases while also increasing the computation time for the 
        %   path planner.
        %
        %   Default: 50
        MaxNumNodes;
    end

    properties(Access = private)
        %MaxNumIterationInternal Maximum number of iterations
        %   This property avoids to go in infinite loop if map does not have 
        %   any free space and unable to sample valid states. This property 
        %   increment by one iteration if valid sample state count is zero 
        %   of any iteration of while loop and come out from this while loop 
        %   if this value reaches to 10. Adopted this behavior from OMPL. 
        %
        %   Default: 10
        MaxNumIterationInternal
    end

    properties (Access = private)

        %graph Handle to MATLAB digraph handle object
        Graph

        %NodesWeightsInternal Adjacency Matrix to stores edges weights  
        NodesWeightsInternal;
    end

    methods
        function obj = plannerPRM(stateSpace, stateValidator, varargin)
            %PLANNERPRM Constructor

            narginchk(2,6);
            obj.validateRequiredInput(stateSpace, stateValidator);
            obj.StateSpace = stateSpace;
            obj.StateValidator = stateValidator;
            
            obj.MaxNumIterationInternal = 10;
            obj.validateOptionalInput(varargin{:});
            
            if (isa(obj.StateSpace, 'stateSpaceSE2') || ...
                    isa(obj.StateSpace, 'stateSpaceDubins') ||...
                    isa(obj.StateSpace, 'stateSpaceReedsShepp') || ...
                    isa(obj.StateSpace, 'manipulatorStateSpace'))
                obj.StateSpace.SkipStateValidation = true;
            end
            
            %Skip state validation for state validators.
            obj.skipStateValidatorValidation();

            %Make graph
            obj.createGraph();

            obj.reset();
        end
    end

    methods

        function [pathObj, solnInfo] = plan(obj, startState, goalState)
            %plan Plan path between start and goal states on roadmap
            %   PATH = plan(PLANNER, STARTSTATE, GOALSTATE) returns an obstacle-free 
            %   path as a navpath object between the start state and the 
            %   goal state within a roadmap that contains a network graph 
            %   of connected nodes.
            %
            %   [PATH, SOLNINFO] = plan(PLANNER, STARTSTATE, GOALSTATE) also 
            %   returns SOLNINFO as a structure that contains the solution 
            %   information of the path planning.
            %   Fields of SOLNINFO:
            %
            %   IsPathFound - Indicates whether a path is found. It returns
            %   as 1 if a path is found. Otherwise, it returns 0.

            % check whether start and goal states are valid
            if ~all(obj.StateValidator.isStateValid(startState)) || ~all(all(isfinite(startState)))
                coder.internal.error('nav:navalgs:plannerprm:StartStateNotValid');
            end

            if ~all(obj.StateValidator.isStateValid(goalState)) || ~all(all(isfinite(goalState)))
                coder.internal.error('nav:navalgs:plannerprm:GoalStateNotValid');
            end

            if all(startState == goalState)
                pathObj                 = navPath(obj.StateSpace, [startState; goalState]);
                solnInfo                = struct();
                solnInfo.IsPathFound    = true;
                return;
            end
            
            %Skip state validation for state validators.
            obj.skipStateValidatorValidation();
            
            %Convert nodes into array
            graphNodes = table2array(obj.Graph.Nodes);
            nNodes = size(graphNodes,1);
            
            %Return empty path if graph does not have any nodes
            if isempty(graphNodes)
                [pathObj, solnInfo] = emptyPath(obj);
                return;
            end
            
            %Find nearest valid graph nodeIds for start and goal states
            start   = obstacleFreeClosetNode(obj, startState, graphNodes, true);
            goal    = obstacleFreeClosetNode(obj, graphNodes, goalState, false);

            %Reset the state validator and state space properties
            obj.reset();
            
            %Return empty path if there is no valid graph node connecting
            %to either start state or goal state
            if isempty(start) || isempty(goal)
                [pathObj, solnInfo] = emptyPath(obj);
                return;
            end

            if(obj.Graph.numedges == 0)
                graphEdges = zeros(0,2);
            else
                graphEdges = table2array(obj.Graph.Edges);
            end

            tempNodesWeights = obj.NodesWeightsInternal;

            %Compute distance from startNode to all Other nodes
            distStart = obj.StateSpace.distance(graphNodes(start,:), graphNodes(1:nNodes,:));

            %Compute distance from edgeLists to goalNode
            distGoal = obj.StateSpace.distance(graphNodes(1:nNodes,:), graphNodes(goal,:));

            %Assign edges distances
            tempNodesWeights(start,:) = distStart;
            tempNodesWeights(:,goal) = distGoal';

            %Find shortest path
            pathIDs = nav.algs.internal.aStar(start, goal, graphEdges', tempNodesWeights);

            if isempty(pathIDs)
                [pathObj, solnInfo] = emptyPath(obj);
                return;
            else
                nodes = table2array(obj.Graph.Nodes);
                path  = nodes(pathIDs',:);

                %Add start state if it's not in path matrix
                if ~isequal(path(1,:), startState)
                    path = [startState; path];
                end

                %Add goal state if it's not in path matrix
                if ~isequal(path(end,:),goalState)
                    path = [path; goalState];
                end

                pathObj = navPath(obj.StateSpace, path);

                solnInfo = struct();
                solnInfo.IsPathFound = true;
            end
        end

        function newObj = copy(obj)
            %copy Create deep copy of planner object
            %   PLANNER2 = copy(PLANNER1) creates a deep copy of the 
            %   specified planner object.

            validator   = obj.StateValidator.copy;
            newObj      = plannerPRM(validator.StateSpace, validator);
            newObj.MaxConnectionDistance = obj.MaxConnectionDistance;
            newObj.MaxNumNodes = obj.MaxNumNodes;
            newObj.MaxNumIterationInternal = obj.MaxNumIterationInternal;
            newObj.NodesWeightsInternal = obj.NodesWeightsInternal;
            newObj.Graph = obj.Graph;
        end
        
        function graph = graphData(obj)
            %graphData Retrieve graph as digraph object
            %   GRAPH = graphData(PLANNER) retrieves graph as a digraph
            %   object.
            
            %Retrieves graph as a digraph object.
            graph = obj.Graph;
        end

    end

    methods(Access = private)
        function createGraph(obj)

            %update Create or update the probabilistic roadmap
            %   update(plannerPRMObj) constructs a roadmap if called for the first
            %   time after plannerPRM object creation. Subsequent calls of update
            %   re-creates the roadmap by re-sampling the Map. The update
            %   function uses StateValidator, MaxNumNodes and MaxConnectionDistance
            %   values to construct the new roadmap.

            maxNumNodes             = obj.MaxNumNodes;
            maxConnectionDistance   = obj.MaxConnectionDistance;

            sampledStates = nan(maxNumNodes, obj.StateSpace.NumStateVariables);
            idx     = 1;
            iter    = 0;
            
            %Execute while loop until either samples 'maxNumNodes' valid
            %states or there is no valid samples found for 10 continues
            %iterations.
            while(1)
                %Sample states using state space uniform sample
                states = obj.StateSpace.sampleUniform(maxNumNodes);

                %Get all new valid states from sampled states.
                isValid = isStateValid(obj.StateValidator, states) == 1;
                vStates = states(isValid,:);

                %Fill new valid states to sampledStates variable
                nVStates = size(vStates,1);
                nRequiredVStates = min(maxNumNodes-idx+1, nVStates);
                sampledStates(idx:idx+nRequiredVStates-1,:) = vStates(1:nRequiredVStates,:);
                
                %Increment iterations if there is no valid state sampled
                %for this particular iteration
                if nVStates == 0
                    iter = iter + 1;
                end

                idx = idx+nRequiredVStates;
                %Exit from while loop if either iter more than 10 or
                %maxNumNode states are sampled
                if iter > obj.MaxNumIterationInternal || idx > maxNumNodes 
                    break;
                end
            end
            
            validStates = sampledStates(~isnan(sampledStates(:,1)),:);

            %Iterate over each nodes and add nodes and edges to
            %plannerGraph
            numNodes = size(validStates,1);

            % Create MATLAB digraph internal objects
            nodeTable = table(validStates, 'VariableNames', {'Nodes'});
            edgeTable = table([zeros(0, 1), zeros(0, 1)], 'VariableNames', {'EndNodes'});
            obj.Graph = digraph(edgeTable, nodeTable);

            if numNodes > 0
                obj.NodesWeightsInternal = inf(numNodes, numNodes);
                
                %Get all possible edges pairs ids for maxNumNodes
                allPossibleEdgesPairsIDs = [repmat((1:numNodes)',numNodes, 1), repelem((1:numNodes)',numNodes,1)];

                %Compute distance for all set of edge pairs
                dist = obj.StateSpace.distance(validStates(allPossibleEdgesPairsIDs(:,1),:),...
                    validStates(allPossibleEdgesPairsIDs(:,2),:));
                [sortedDist, label] = sort(dist);
                
                %Ignore self-loop as well as edges which have distance more 
                %than maxConnectionDistance
                validDistIDs    = sortedDist <= maxConnectionDistance & sortedDist ~= 0;
                validDists      = sortedDist(validDistIDs);
                validLabels     = label(validDistIDs);
    
                for j=1:length(validDists)
                    newNodeID       = allPossibleEdgesPairsIDs(validLabels(j),1);
                    newNode         = validStates(newNodeID,:);
                    nearByNodeID    = allPossibleEdgesPairsIDs(validLabels(j),2);
                    nearByNode      = validStates(nearByNodeID,:);
                    
                    %Assign edge weight to adjacency matrix, NodesWeightsInternal
                    obj.NodesWeightsInternal(newNodeID,nearByNodeID) = sortedDist(j);
                    
                    %Add an edge from newNode to nearByNode to Graph if 
                    %these two nodes are not in same component and path 
                    %is valid from newNode to nearByNode.
                    compIDs = conncomp(obj.Graph);
                    isCompIDsSame = isequal(compIDs(newNodeID), compIDs(nearByNodeID));
    
                    if  ~isCompIDsSame && isMotionValid(obj.StateValidator, newNode, nearByNode)
                        obj.Graph = obj.Graph.addedge(newNodeID, nearByNodeID);
                    end
                end
            end
        end

        function [pathObj, solnInfo] = emptyPath(obj)
            %emptyPath Construct empty path

            pathObj                 = navPath(obj.StateSpace);
            solnInfo                = struct();
            solnInfo.IsPathFound    = false;
        end

        function skipStateValidatorValidation(obj)
            %cleanUp Skip validaton for state validators
            
            if isa(obj.StateValidator, 'manipulatorCollisionBodyValidator')
                obj.StateValidator.SkipStateValidation = true;
            elseif isa(obj.StateValidator, 'validatorOccupancyMap')
                obj.StateValidator.SkipStateValidation = true;
                obj.StateValidator.configureValidatorForFastOccupancyCheck();
            end
        end

        function reset(obj)
            %reset Reset some flags after finding valid nearest neighbor 
            %nodes from start and goal states
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
    end

    methods (Access = private)
        function validateRequiredInput(obj, ss, sv)
            %validateRequiredInput

            validateattributes(ss, {'nav.StateSpace'}, {'scalar', 'nonempty'}, 'plannerPRM', 'stateSpace');
            validateattributes(sv, {'nav.StateValidator'}, {'scalar', 'nonempty'}, 'plannerPRM', 'stateValidator');

            if coder.target('MATLAB')
                if ss == sv.StateSpace % reference to the same state space object
                    obj.StateValidator  = sv.copy;
                    obj.StateSpace      = ss.copy;
                else
                    coder.internal.error('nav:navalgs:plannerprm:RequireSameStateSpace');
                end
            else
                % ignore the handle check during codegen
                obj.StateValidator  = sv.copy;
                obj.StateSpace      = ss.copy;
            end
        end

        function validateOptionalInput(obj, varargin)
            %validateOptionalInput

            name = {'MaxNumNodes', 'MaxConnectionDistance'};

            % Create default value cell array
            default = {50, inf};

            % Create a parser
            parser = robotics.core.internal.NameValueParser(name,default);

            % Parse name-value inputs (where the name-value inputs are
            % contained in varargin).
            parse(parser, varargin{:});

            % Validate 'MaxNumNodes'
            maxNumNodes                 = parameterValue(parser, "MaxNumNodes");
            validateattributes(maxNumNodes, {'double'}, {'scalar', 'nonnan', 'finite', 'integer', ...
                'nonempty', 'positive'},'plannerPRM','MaxNumNodes');
            obj.MaxNumNodes = maxNumNodes;

            % Validate 'MaxConnectionDistance'
            maxConnectionDistance       = parameterValue(parser, "MaxConnectionDistance");
            validateattributes(maxConnectionDistance, {'double'}, ...
                {'scalar', 'real', 'nonempty', 'positive', 'nonnan'},...
                'plannerPRM','MaxConnectionDistance');
            obj.MaxConnectionDistance           = maxConnectionDistance;
        end

        function closestObstacleFreeNodeID = obstacleFreeClosetNode(obj, fromNode, toNode, isInward)
            %closestObstacleFreeState

            dist = obj.StateSpace.distance(fromNode, toNode);
            [sortedDist, labels] = sort(dist);
            
            %Avoid self loop
            validDistIDs    = sortedDist ~= 0;
            validDists      = sortedDist(validDistIDs);
            validLabels     = labels(validDistIDs);

            closestObstacleFreeNodeID = [];
            %Find nearest valid state
            for j=1:length(validDists)
                nearByNodeID = validLabels(j);

                if isInward
                    isValid = isMotionValid(obj.StateValidator, fromNode, toNode(nearByNodeID,:));
                else
                    isValid = isMotionValid(obj.StateValidator, fromNode(nearByNodeID,:), toNode);
                end

                if isValid
                    closestObstacleFreeNodeID = nearByNodeID;
                    return;
                end
            end
        end
    end
end