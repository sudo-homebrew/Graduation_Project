classdef plannerAStarGrid < nav.algs.internal.InternalAccess
%This class is for internal use only. It may be removed in the future.

%plannerAStarGrid Interface to internal implementation used for A* algorithm for
%grid maps.
%
%   This class is a collection of functions used for finding
%   path in grid maps using AStar algorithm. Its main purpose is to
%   dispatch function calls correctly using wrappers when executed in
%   MATLAB.
%
%   During MATLAB execution, we call internal builtins functions via this class.
%   During code generation we use a codegen-compatible version and redirect to
%   nav.algs.internal.codegen.plannerAStarGrid.
%
%   astar = nav.algs.internal.plannerAStarGrid(Map,Row,Col,Zlim,...
%                   ObstacleThreshold,MapResolution,varargin);
%   'varargin' contains NameValue Pair for 'GCostFcn' and 'HCostFcn'
%
%   Input:
%         Map :                 Grid map as 2d double vector.
%         Row :                 Number of rows in map
%         Col :                 Number of columns in map
%         Zlim:                 Number of depth layers in map. (Set as 1)
%         ObstacleThreshold:    Value above which the grid is considered as
%                               obstacle.
%         MapResolution:        Resolution of map in Grid cells per meter.
%         GCostFcn:             Function handle for custom gcost function.
%         HCostFcn:             Function handle for custom heuristic cost function.
%
%   See also plannerAStarGrid

% Copyright 2019-2020 The MathWorks, Inc.

%#codegen

    properties (Access = {?nav.algs.internal.InternalAccess})
        % Pointer of A* object from standard builtins in MATLAB
        AStarObjPointer
        % Grid map, 2d matrix.
        Gridmap
        % Threshold above which the point is considered as an obstacle.
        OccupiedThreshold
    end

    methods
        function obj = plannerAStarGrid(map,row,col,zlim,obsThresh,mapRes,varargin)
        % Constructor for internal plannerAStarGrid, which sets standard
        % builtins parameters.

            narginchk(6,10)
            % Sets AStarObjPointer to builtins class.
            obj.AStarObjPointer = nav.algs.internal.AStarMcos(map,row,col,zlim,obsThresh,mapRes);
            obj.Gridmap = map;
            obj.OccupiedThreshold=obsThresh;
            if(nargin>6)
                %% Name Value parser
                % Create parameter name cell array
                names = {'GCostFcn','HCostFcn'};

                % Create default value cell array
                defaults = {@obj.Euclidean,@obj.Euclidean};

                % Create a parser
                parser = robotics.core.internal.NameValueParser( ...
                    names,defaults);

                % Parse name-value inputs (where the name-value inputs are
                % contained in varargin).
                parse(parser, varargin{1:(nargin-6)});

                % Set Custom gcost function for internal builtins class.
                GCostFcn = parameterValue(parser,'GCostFcn');
                if(~isequal(func2str(GCostFcn),func2str(defaults{1})))
                    obj.AStarObjPointer.configureGFcn(GCostFcn);
                end
                % Set Custom heuristic cost function for internal builtins class.
                HCostFcn = parameterValue(parser,'HCostFcn');
                if(~isequal(func2str(HCostFcn),func2str(defaults{2})))
                    obj.AStarObjPointer.configureHFcn(HCostFcn);
                end
            end
        end

        function path = plan(obj,startIn,goalIn)
        %plan Dispatch plan call
        % plan() finds the path between the start and goal. startIn and
        % goalIn are row/column of the grid.

        % Validation for start and goal
            validateattributes(startIn,...
                               {'double'}, {'integer','positive','vector','numel',2}, 'AStar', 'Start');
            validateattributes(goalIn,...
                               {'double'}, {'integer','positive','vector','numel',2}, 'AStar', 'Goal');

            % Converting startIn and goalIn to 1x3 format, giving default
            % index as '1'
            start = ones(1,3);
            goal = ones(1,3);

            start(1:2) = startIn;
            goal(1:2) = goalIn;
            % Shifting to C++ indexing format, which starts from '0'
            start = start -1;
            goal = goal -1;

            % Call standard builtin in MATLAB
            path = obj.AStarObjPointer.plan(start,goal);

            % Shifting to MATLAB indexing format, which starts from '1'
            path = path+1;
        end
    end

    methods(Hidden)
        % Following functions are for internal purpose only and to be used
        % majorly for debugging and hence are marked hidden.

        function setHeuristicMethod(obj,distMethod)
        % setHeuristicMethodObject Dispatch setHeuristicMethodObject call
        % to set the builtin heuristic cost function for A* object.
            [~,distMethodVal]= nav.internal.validation.validateAStarBuiltinCostFunction(distMethod);

            % Call standard builtin in MATLAB
            obj.AStarObjPointer.HeuristicMethod=distMethodVal;        
	end
        function str = getHeuristicMethod(obj)
            [~,~,ValidStringsDist]=nav.internal.validation.validateAStarBuiltinCostFunction('Eu');
            str = ValidStringsDist{obj.AStarObjPointer.HeuristicMethod};
        end

        function setGCostMethod(obj,distMethod)
        % setGCostMethod Dispatch setGCostMethod call to set the builtin
        % gcost function for A* object.
            [~,distMethodVal]= nav.internal.validation.validateAStarBuiltinCostFunction(distMethod);
            
            % Call standard builtin in MATLAB
            obj.AStarObjPointer.GCostMethod=distMethodVal;
        end
        function str = getGCostMethod(obj)
            [~,~,ValidStringsDist]=nav.internal.validation.validateAStarBuiltinCostFunction('Eu');            
            str = ValidStringsDist{obj.AStarObjPointer.GCostMethod};
        end

        function setTieBreakerConstant(obj,tiebreaker)
        % setTieBreakerConstantObject Dispatch TieBreakerConstant call which
        % sets the tie breaker value for A* object.
        % Call standard builtin in MATLAB
            obj.AStarObjPointer.TieBreakerConstant=tiebreaker;

        end
        function val = getTieBreakerConstant(obj)
        % Call standard builtin in MATLAB
            val = obj.AStarObjPointer.TieBreakerConstant;
        end

        function start = getStart(obj)
        % getStart Dispatch getStart call and returns start point.
        % Call standard builtin in MATLAB
            start = obj.AStarObjPointer.getStart();

            % Shifting to MATLAB indexing format, which starts from '1'
            start = start+1;
        end

        function goal = getGoal(obj)
        % getGoal Dispatch getGoal call and returns goal point.
        % Call standard builtin in MATLAB
            goal = obj.AStarObjPointer.getGoal();

            % Shifting to MATLAB indexing format, which starts from '1'
            goal = goal+1;
        end

        function path = getPathIndices(obj)
        % getPathIndices Dispatch getPathIndices call and returns node
        % indices of path.
        % Call standard builtin in MATLAB
            path = obj.AStarObjPointer.getPathIndices();
        end

        function pathRC = getPath(obj)
        % getPath Dispatch getPath call and returns Path as grid points.
        % Call standard builtin in MATLAB
            pathRC = obj.AStarObjPointer.getPath();

            % Shifting to MATLAB indexing format, which starts from '1'
            pathRC = pathRC+1;
        end

        function nodes = getNodesExploredIndices(obj)
        % getNodesExploredIndices Dispatch getNodesExploredIndices call and
        % returns indices of explored nodes of grid.
        % Call standard builtin in MATLAB
            nodes = obj.AStarObjPointer.getNodesExploredIndices();
        end

        function nodes = getNodesExplored(obj)
        % getNodesExplored Dispatch getNodesExplored call and returns all
        % the explored nodes of grid in [row col] format.
        % Call standard builtin in MATLAB
            nodes = obj.AStarObjPointer.getNodesExplored();

            % Shifting to MATLAB indexing format, which starts from '1'
            nodes=nodes+1;
        end

        function numNodes = getNumNodesExplored(obj)
        % getNumNodesExplored Dispatch getNumNodesExplored call and returns
        % number of nodes explored during the search.
        % Call standard builtin in MATLAB
            numNodes = obj.AStarObjPointer.getNumNodesExplored();
        end

        function cost = getPathCost(obj)
        % getPathCost Dispatch getPathCost call and returns the cost of the
        % found shortest path.
        % Call standard builtin in MATLAB
            cost = obj.AStarObjPointer.getPathCost();
        end

        function gcostMat = getGCostMatrix(obj)
        % getGCostMatrix Dispatch getGCostMatrix call and returns the gcost
        % of each explored node from start grid.
        % Call standard builtin in MATLAB
            gcostMat = obj.AStarObjPointer.getGCostMatrix();
        end
    end
    
    methods (Static = true, Access = private)
        function name = matlabCodegenRedirect(~)
        % Redirects the calls to given class, when the target is
        % 'coder'
            name = 'nav.algs.internal.codegen.plannerAStarGrid';
        end
    end
end

% LocalWords:  astar GCostFcn HCostFcn gcost Pointer GCost
