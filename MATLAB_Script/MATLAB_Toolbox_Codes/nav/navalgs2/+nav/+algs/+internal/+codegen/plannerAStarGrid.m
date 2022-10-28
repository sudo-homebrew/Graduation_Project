classdef plannerAStarGrid <  nav.algs.internal.InternalAccess
% nav.algs.internal.codegen.plannerAStarGrid Class implementing the AStar
% algorithm for grid maps that is compatible with code generation

% Copyright 2020-2021 The MathWorks, Inc.

%#codegen
    properties
        % Grid based representation of world.
        Map
        % Function handle of custom heuristic cost.
        HCostFcn
        % Function handle of custom gcost.
        GCostFcn
        % Tie breaker to nudge heuristics a little to break ties.
        TieBreaker = 1
    end

    properties (Access = private)
        % Dimensions of map.
        Row
        Col
        Zlim

        % Threshold above which a grid is considered as obstacle.
        ObstacleThreshold
        % Resolution of map in cells per meter.
        MapResolution

        % Start position
        Start
        % Goal position
        Goal

        % Indices of path points from given start to goal.
        PathIndices
        % Path points in grid of path found by A* algorithm for given start to goal.
        Path
        % Cost of path found by A* algorithm.
        PathCost

        % Indices of explored nodes during search.
        NodesExploredIndices
        % Grid points of explored nodes during search.
        NodesExplored
        % Number of nodes explored.
        NumNodesExplored

        % Matrix which stores gcost from start grid to each explored node.
        GCostMatrix
        % Stores col position of parent of a cell in grid
        ParentCol
        % Stores row position of parent of a cell in grid
        ParentRow
        % Stores indices of all grid cells mapped to their position.
        MapIndex
        % Stores Position of all grid cells mapped to their indices.
        AllNodes

        % Flag for using custom cost function.
        UseCustomG=0;
        UseCustomH=0;
        % Enumerated value for inbuilt cost function to be used.
        GCostMethod=1;
        HCostMethod=1;
    end
    
    methods
        function obj =  plannerAStarGrid(map,row,col,zlim,obstacleThreshold,mapResolution,varargin)
            
            % To verify that no more than 10 elements are passed to
            % function.
            narginchk(6,10);
            
            obj.Map = map;
            obj.Row = row;
            obj.Col = col;
            obj.Zlim = zlim;
            % Verifies if map dimensions are passed correctly.
            if(size(obj.Map,1)~=row||size(obj.Map,2)~=col||zlim~=1)
                coder.internal.error('nav:navalgs:plannerastargrid:OccupiedLocation', 'incorrectInputDimension');
            end
            obj.ObstacleThreshold = obstacleThreshold;
            obj.MapResolution = mapResolution;

            % Initializing default parameters.
            obj.ParentCol=double(zeros(row,col));
            obj.ParentRow=double(zeros(row,col));
            obj.MapIndex = reshape(1:numel(map),size(map));
            [allRow,allCol]=ind2sub(size(map),1:numel(map));
            obj.AllNodes = [allRow',allCol'];

            % Name Value parser
            % Create parameter name cell array
            names = {'GCostFcn','HCostFcn'};

            % Create default value cell array
            defaults = {@obj.Euclidean,@obj.Euclidean};

            % Create a parser
            parser = robotics.core.internal.NameValueParser( ...
                names,defaults);

            % Parse name-value inputs (where the name-value inputs are
            % contained in varargin).
            parse(parser, varargin{1:end});

            obj.GCostFcn = parameterValue(parser,'GCostFcn');
            if(~isequal(func2str(obj.GCostFcn),func2str(defaults{1})))
                obj.UseCustomG=1;
            end
            obj.HCostFcn = parameterValue(parser,'HCostFcn');
            if(~isequal(func2str(obj.HCostFcn),func2str(defaults{2})))
                obj.UseCustomH=1;
            end

        end

        function OptimalPath = plan(obj,startIn,goalIn)
        %plan Finds path between the start and goal using A* algorithm.


            map = obj.Map;
            row = obj.Row;
            col = obj.Col;
            % Matrix keeping track of G-scores
            GScore=inf(row,col);
            % Matrix keeping track of F-scores (only open list)
            FScore=inf(row,col);
            % Matrix keeping of explored grid cells
            ExploredNodes=int8(zeros(row,col));
            % Local Matrix keeping track of col position of parent in grid
            ParentCol1=double(zeros(row,col));
            % Local Matrix keeping track of row position of parent in grid
            ParentRow2=double(zeros(row,col));
            % Matrix keeping track of closed/explored grid cells
            ClosedMAT=int8(zeros(row,col));
            % Adding object-cells to closed matrix
            ClosedMAT(map==1)=1;
            % Creating priority queue for OpenList or to be explore nodes.
            OpenList = nav.algs.internal.PriorityQueue(5, 3);

            % Setting default path, when no path is found.
            obj.Path = 0;
            obj.PathIndices = [1 1];

            % Generating matrix of heuristic cost of every grid cell from
            % goal grid.
            Hn2 = getHeuristicMatrix(obj,goalIn);
            Hn = double(reshape(Hn2,row,col));

            % Setting start node with FScore as heuristic and opening first node.
            FScore(startIn(1),startIn(2))=Hn(startIn(1),startIn(2));
            OpenList.push([obj.MapIndex(startIn(1),startIn(2)),obj.MapIndex(startIn(1),startIn(2)),double(FScore(startIn(1),startIn(2))),0,double(Hn(startIn(1),startIn(2)))]);
            ExploredNodes(startIn(1),startIn(2))=1;
            GScore(startIn(1),startIn(2))=0;

            % Useful for codegen
            CurrentRow=0;
            CurrentCol=0;

            % Getting neighbor list for adjacent points to current grid
            % cell.
            [Neighbors,NumNeighbors] = obj.getNeighbors();
            while (1) % Code will break when path found or when no path exist

                % Fetching the element with minimum fScore of priority queue
                % to explore.
                MinScoreNode = OpenList.top();

                if MinScoreNode(3)==inf
                    break;
                end
                if OpenList.isEmpty()
                    %OpenList get Exhausted and path not found!!
                    obj.ParentCol=startIn(2);
                    obj.ParentRow=startIn(1);
                    obj.Path=[];
                    obj.PathIndices=[];
                    break;
                end

                CurrentRow = obj.AllNodes(MinScoreNode(1),1);
                CurrentCol = obj.AllNodes(MinScoreNode(1),2);

                if MinScoreNode(1)==obj.MapIndex(goalIn(1),goalIn(2))
                    % Goal reached and path found.
                    obj.ParentCol=ParentCol1;
                    obj.ParentRow=ParentRow2;
                    % reconstructPath called to consolidate path to return.
                    [obj.Path,obj.PathIndices]= reconstructPath(obj,CurrentRow,CurrentCol,startIn);
                    break;
                end

                % Removing node from OpenList to ClosedList
                OpenList.pop();
                ClosedMAT(CurrentRow,CurrentCol)=1;

                % openToPush is used for performance improvement to avoid
                % multiple push of same data  to OpenList.
                openToPush = zeros(NumNeighbors,5);
                openToPushK=1;
                for p=1:NumNeighbors
                    % Iterating over each neighbors.
                    i=Neighbors(p,1); % Row
                    j=Neighbors(p,2); % Col

                    if isValid(obj,CurrentRow,CurrentCol,row,col,i,j)
                        % Validates if current node is free.
                        continue;
                    end

                    if(ClosedMAT(CurrentRow+i,CurrentCol+j)~=0)
                        % Validates if Neighbor is open for exploration.
                        continue;
                    end

                    if passObstacles(obj,CurrentRow,CurrentCol,i,j)
                        % Validates that Neighbor pass through obstacle;
                        continue;
                    end
                    % End of basic validation of neighboring cell.

                    % Computing actual cost of moving from current node to
                    % neighboring node.
                    gcost = gcostValue(obj,CurrentRow,CurrentCol,i,j);
                    tentative_gScore = GScore(CurrentRow,CurrentCol) + gcost(1);

                    if FScore(CurrentRow+i,CurrentCol+j)==inf
                        % Checks if node is already explored, if not it
                        % adds to OpenList, and updates cost.
                        ExploredNodes(CurrentRow+i,CurrentCol+j)=1;
                        ParentCol1(CurrentRow+i,CurrentCol+j)=CurrentCol;
                        ParentRow2(CurrentRow+i,CurrentCol+j)=CurrentRow;
                        GScore(CurrentRow+i,CurrentCol+j)=tentative_gScore;
                        FScore(CurrentRow+i,CurrentCol+j)= tentative_gScore+Hn(CurrentRow+i,CurrentCol+j);
                        a = obj.MapIndex(CurrentRow+i,CurrentCol+j);
                        b = obj.MapIndex(CurrentRow,CurrentCol);

                        openToPush(openToPushK,:) = [a,...
                                            b,...
                                            double(FScore(CurrentRow+i,CurrentCol+j)),...
                                            double(GScore(CurrentRow+i,CurrentCol+j)),...
                                            double(Hn(CurrentRow+i,CurrentCol+j))];
                        OpenList.push(openToPush(openToPushK,:));
                        openToPushK=openToPushK+1;

                    elseif tentative_gScore >= GScore(CurrentRow+i,CurrentCol+j)
                        % New exploration path is longer than already
                        % explored way, so continuing to next neighbor.
                        continue
                    else
                        % New exploration path is shorter than already
                        % present path. Updating the cost and parent
                        % pointer.
                        ParentCol1(CurrentRow+i,CurrentCol+j)=CurrentCol;
                        ParentRow2(CurrentRow+i,CurrentCol+j)=CurrentRow;
                        GScore(CurrentRow+i,CurrentCol+j)=tentative_gScore;
                        FScore(CurrentRow+i,CurrentCol+j)= tentative_gScore+Hn(CurrentRow+i,CurrentCol+j);
                    end
                end
            end
            % Consolidating all the information to return for debug and
            % testing purposes.
            obj.PathCost = GScore(CurrentRow,CurrentCol);
            obj.GCostMatrix = reshape(GScore,numel(GScore),1);
            obj.Start = startIn;
            obj.Goal = goalIn;
            obj.NodesExploredIndices = find(ExploredNodes==1);
            obj.NumNodesExplored = size(obj.NodesExploredIndices,1);
            obj.NodesExplored = obj.AllNodes(obj.NodesExploredIndices,:);
            OptimalPath = obj.Path;
        end

        function [OptimalPath,pathIndices] = reconstructPath(obj,CurrentRow,CurrentCol,startIn)
        %reconstructPath Follows through the parent pointers to retrace the
        %path from current node to start.
            OptimalPath=zeros(numel(obj.Map),2);
            pathIndices = zeros(numel(obj.Map),1);
            k = 1;
            OptimalPath(1,:)=[CurrentRow CurrentCol];
            pathIndices(1)= obj.MapIndex(CurrentRow,CurrentCol);
            while (1)
                k=k+1;
                CurrentColDummy=obj.ParentCol(CurrentRow,CurrentCol);
                CurrentRow=obj.ParentRow(CurrentRow,CurrentCol);
                CurrentCol=CurrentColDummy;
                if(CurrentRow==0||CurrentCol==0)
                    k = k-1;
                    break;
                end
                OptimalPath(k,:)=[CurrentRow CurrentCol];
                pathIndices(k)= obj.MapIndex(CurrentRow,CurrentCol);
                if (((CurrentCol== startIn(2))) &&(CurrentRow==startIn(1)))
                    break
                end
            end
            OptimalPath = flip(OptimalPath(1:k,:),1);
            pathIndices = flip(pathIndices(1:k,:),1);
        end

        function setHeuristicMethod(obj,distMethod)
        % setHeuristicMethod Sets the heuristic cost to builtin function.
            [~,distMethodVal]= nav.internal.validation.validateAStarBuiltinCostFunction(distMethod);
            obj.HCostMethod = distMethodVal;
            obj.UseCustomH=0;
        end
        function heuristicMethod = getHeuristicMethod(obj)
        % getHeuristicMethod Returns builtin cost function used.
            [~,~,ValidStringsDist]=nav.internal.validation.validateAStarBuiltinCostFunction('Eu');
            heuristicMethod = ValidStringsDist{obj.HCostMethod};
        end

        function setGCostMethod(obj,distMethod)
        % setGCostMethod Sets builtin gcost function.
            [~,distMethodVal]= nav.internal.validation.validateAStarBuiltinCostFunction(distMethod);
            obj.GCostMethod = distMethodVal;
            obj.UseCustomG=0;
        end
        function gcostMethod = getGCostMethod(obj)
        % getGCostMethod Returns builtin gcost function.
            [~,~,ValidStringsDist]=nav.internal.validation.validateAStarBuiltinCostFunction('Eu');
            gcostMethod = ValidStringsDist{obj.GCostMethod};            
        end

        function setTieBreakerConstant(obj,val)
        % setTieBreakerConstant Sets the constant multiplier to break ties.
            obj.TieBreaker = val;
        end
        function tieBreakVal = getTieBreakerConstant(obj)
        % getTieBreakerConstant Returns the value of tiebreaker used.
            tieBreakVal =  obj.TieBreaker;
        end

        function start = getStart(obj)
        % getStart Returns start point of last search
            start = obj.Start;
        end

        function goal = getGoal(obj)
        % getGoal Returns goal point of last search
            goal = obj.Goal;
        end

        function pathIndices = getPathIndices(obj)
        % getPathIndices Returns indices of shortest path found using A*.
            pathIndices = obj.PathIndices;
        end

        function pathRowCol = getPath(obj)
        % getPath Returns grid position of shortest path found using A*.
            pathRowCol = obj.Path;
        end

        function nodesIndices = getNodesExploredIndices(obj)
        % getNodesExploredIndices Returns node indices of explored nodes
        % during A* search.
            nodesIndices = obj.NodesExploredIndices;
        end

        function nodesRowCol = getNodesExplored(obj)
        % getNodesExplored Returns grid cells of nodes explored during A*
        % search.
            nodesRowCol = obj.NodesExplored;
        end

        function numNodes = getNumNodesExplored(obj)
        % getNumNodesExplored Returns number of explored nodes during
        % search.
            numNodes = obj.NumNodesExplored;
        end

        function cost = getPathCost(obj)
        % getPathCost Returns cost of shortest path found.
            cost = obj.PathCost;
        end

        function gcostmat = getGCostMatrix(obj)
        % getGCostMatrix Returns matrix of gcost of each explored node from
        % start grid.
            gcostmat = obj.GCostMatrix;
        end
    end

    methods(Access = private)
        function [Neighbors,NumNeighbors] = getNeighbors(~)
        %getNeighbors Finds the neighboring cells which are from
        %ConnectingDistance away from the current cell. It assumes current
        %grid cell to be [0,0].
            ConnectingDistance = 1;
            NeighborCheck=ones(2*ConnectingDistance+1);
            Corners=2*ConnectingDistance+2;
            Mid=ConnectingDistance+1;
            for i=1:ConnectingDistance-1
                NeighborCheck(i,i)=0;
                NeighborCheck(Corners-i,i)=0;
                NeighborCheck(i,Corners-i)=0;
                NeighborCheck(Corners-i,Corners-i)=0;
                NeighborCheck(Mid,i)=0;
                NeighborCheck(Mid,Corners-i)=0;
                NeighborCheck(i,Mid)=0;
                NeighborCheck(Corners-i,Mid)=0;
            end
            NeighborCheck(Mid,Mid)=0;
            [row, col]=find(NeighborCheck==1);
            Neighbors=[row col]-(ConnectingDistance+1);
            NumNeighbors=size(col,1);
        end
        function cost = distance(obj,pose1,pose2,type)
        %distance Returns the cost between pose1 and pose2 using the method
        % mentioned by parameter 'type'.
            switch(type)
              case 2
                cost = obj.Manhattan(pose1,pose2);
              case 3
                cost = obj.Chebyshev(pose1,pose2);
              case 4
                cost = obj.EuclideanSquared(pose1,pose2);
              otherwise
                cost = obj.Euclidean(pose1,pose2);
            end
            cost = cost /obj.MapResolution;
        end

        function Hn = getHeuristicMatrix(obj,goalIn)
        %getHeuristicMatrix Computes the matrix of heuristic cost of each
        %grid cell from goal point, either by using builtin function or
        %using provided custom function handle.
            Hn = zeros(size(obj.AllNodes,1),1);

            if(obj.UseCustomH==0)
                Hn = obj.TieBreaker*obj.distance(obj.AllNodes,repmat(goalIn,numel(obj.Map),1),obj.HCostMethod);
            else
                for i=1:size(obj.AllNodes,1)
                    hcost = obj.TieBreaker*obj.HCostFcn(obj.AllNodes(i,:),goalIn)/obj.MapResolution;
                    obj.validateCustomCostOutput(hcost);
                    if(~isempty(hcost))
                        Hn(i) = hcost(1);
                    end
                end
            end
        end

        function gcostNeighbor = gcostValue(obj,CurrentRow,CurrentCol,i,j)
        %gcostValue Computes the gcost between the current grid to neighboring
        % grid cell, either by using builtin function or using provided
        % custom function handle.
            if (obj.UseCustomG==0)
                gcostNeighbor = obj.distance([0 0],[i j],obj.GCostMethod);
            else
                gcostNeighbor = obj.GCostFcn([CurrentRow,CurrentCol],[CurrentRow+i,CurrentCol+j])/obj.MapResolution;
                obj.validateCustomCostOutput(gcostNeighbor);           
            end
        end
        function validateCustomCostOutput(~,val)
            if(isempty(val))
                coder.internal.error('nav:navalgs:plannerastargrid:UserDistanceFunctionReturnsEmpty');
            elseif(size(val,2)>1||~isa(val,'double'))
                coder.internal.error('nav:navalgs:plannerastargrid:InvalidCostFunctionHandleOutput');
            elseif(isnan(val(1))||isinf(val(1)))
                coder.internal.error('nav:navalgs:plannerastargrid:UserDistanceFunctionReturnsNaNOrInf');
            end            
        end
        
        function flag = isValid(obj,CurrentRow,CurrentCol,row,col,i,j)
        %isValid Validates that Neighboring grid is in map bound and is
        %free from obstacle.
            if CurrentRow+i<1||CurrentRow+i>row||CurrentCol+j<1||CurrentCol+j>col||obj.Map(CurrentRow+i,CurrentCol+j)>=obj.ObstacleThreshold
                flag = 1;
            else
                flag = 0;
            end
        end
        function flag = passObstacles(obj,CurrentRow,CurrentCol,i,j)
        % Verifies if there is obstacle between the current grid and
        % neighboring grid. Useful when we are skipping steps.
            flag = 0;
            if (abs(i)>1||abs(j)>1)
                % Need to check that the path does not pass an object
                JumpCells=2*max(abs(i),abs(j))-1;
                for K=1:JumpCells
                    YPOS=round(K*i/JumpCells);
                    XPOS=round(K*j/JumpCells);

                    if (obj.Map(CurrentRow+YPOS,CurrentCol+XPOS)==1)
                        flag =1;
                        break;
                    end
                end
            end
        end
    end
    methods(Static)
        function dist = Chebyshev(pose1,pose2)
            difference = abs(bsxfun(@minus, pose1, pose2));
            dist = sum(difference,2) -  min(difference(:,1), difference(:,2));
        end
        function dist = Euclidean(pose1,pose2)
            dist = sqrt(sum((pose1-pose2).^2,2));
        end
        function dist = EuclideanSquared(pose1,pose2)
            dist = sum((pose1-pose2).^2,2);
        end
        function dist = Manhattan(pose1,pose2)
            dist = sum(abs(bsxfun(@minus, pose1, pose2)),2);
        end
    end
end

% LocalWords:  gcost astar HCost GCostFcn FScore getGCostMatrix setGCostMethod getGCostMethod
