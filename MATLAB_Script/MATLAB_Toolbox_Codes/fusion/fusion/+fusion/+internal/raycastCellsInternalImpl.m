function [isCollisionFree, collisionGridPt, endPtsVec, midPtsVec]  = raycastCellsInternalImpl(p1, p2, varargin)
%This function is for internal use only. It may be removed in the future.

%RAYCASTINTERNAL Test a set of lines for collisions or return all cells along a set of rays
%
%   [ISCOLLISIONFREE, COLLISIONPT, ENDPTS, MIDDLEPTS] = raycastCellsInternalImpl(P1, P2,  ROWS, COLS, RES, LOC)
%   returns cells along the laser beams that connect the XY point P1 to each
%   XY pair in P2.
%   The ENDPTS represent indices of cells touching all end points in P2
%   and the MIDDLEPTS represent all cells between P1 and P2, excluding ENDPTS. 
%   This function call does not check for collisions. ISCOLLISIONFREE is
%   always an N-element true vector and COLLISIONPT is N-by-2 zero-vector,
%   where N is the number of XY pairs in P2.
%
%   [ISCOLLISIONFREE, COLLISIONPT, ENDPTS, MIDDLEPTS] = raycastCellsInternalImpl(P1, P2,  ROWS, COLS, RES, LOC, RANGEISMAX)
%   returns cells along the laser beams that connect P1 to each xy point in P2. 
%   If RANGEISMAX(i) is true, the last point in the ray is considered to be
%   a 'miss', and is not included in ENDPTS.
%   This function call does not check for collisions. ISCOLLISIONFREE is
%   always an N-element true vector and COLLISIONPT is N-by-2 zero-vector
%   where N is the number of XY pairs in P2.
%
%   [ISCOLLISIONFREE, COLLISIONPT, ENDPT, MIDDLEPTS] = raycastCellsInternalImpl(P1, P2, MAP, RES, LOC)
%   checks for collisions with occupied cells (true cells) for line
%   segment(s) P1=[X1,Y1] to P2(i)=[X2,Y2]. Elements of ISCOLLISIONFREE are 
%   true if there is no collision with obstacles, and false otherwise. 
%   COLLISIONPT contains the first grid cell location [X,Y], where each ray
%   intersects an occupied map cell. The ENDPTS and MIDDLEPTS are not 
%   computed and are returned as zeros.
%
%   Endpoints are in world coordinate system and
%   can be floating point values. ROWS and COLS are map size in terms of
%   rows and columns, MAP is an N-by-M matrix of logicals, RES
%   is the resolution of the grid cells in cells per meter and LOC is the
%   location of the lower left corner of the grid in the world frame.
%   RANGEISMAX is an N-element Boolean vector.
%   Input P1 (1x2) and P2 (Nx2) are vectors representing points on the grid.
%   X is Column index and Y is row index. 
%   When multiple end-points are provided, results for each ray are vertically
%   combined to form the outputs isCollisionFree, collisionGridPt,
%   endPtsVec, and midPtsVec.
%   This algorithm is known as Digital Differential Analyzer (or DDA).

%   Copyright 2014-2019 The MathWorks, Inc.
%
%   Reference:
%   [1] "ARTS: Accelerated Ray-Tracing System,", Fujimoto, A.; Tanaka, T.;
%       Iwata, K., Computer Graphics and Applications, IEEE , vol.6, no.4,
%       pp.16,26, April 1986

%#codegen

    narginchk(5,7);
    
    % Get number of incoming rays
    numRays = size(p2,1);
    
    % Get the grid size. The algorithm operates in X, Y. Map inputs are Row-
    % Columns.
    if nargin == 5
        map = varargin{1};
        gridSize = [size(map, 2), size(map, 1)];
        resolution = varargin{2};
        gridLocation = varargin{3};
    else
        %rows = varargin{1}, columns = varargin{2}
        gridSize = [varargin{2}, varargin{1}];
        resolution = varargin{3};
        gridLocation = varargin{4};
        
        if nargin == 7
            rangeIsMax = varargin{5};
        else
            rangeIsMax = false(numRays,1);
        end
    end
    
    % Find xy distance for longest ray
    dMax = max(abs([p2(:,1)-p1(1); p2(:,2)-p1(2)]));
    
    % Allocate vector capable of storing N max length rays. For an 
    % individual ray, the maximum number of visited cells occurs
    % when the ray begins and ends on the corner of a cell and moves
    % diagonally. In this scenario, each time the ray intersects with the 
    % grid the intersection occurs at the boundary of 4 cells, but all 
    % points except the first have 1 cell overlap. All other scenarios
    % will intersect with fewer cells, so for N-rays, we allocate the
    % following:
        
    maxCells = 4+3*ceil(resolution*dMax);
    midPtsVec = zeros(maxCells*numRays,2);
    endPtsVec = zeros(numRays*4,2);
    curEndIdx = 1;
    curMidIdx = 1;
    
    % Allocate output vector for returning collision information for each ray
    collisionGridPt = nan(numRays,2);
    isCollisionFree = true(numRays,1);
    
    % Shift and adjust the coordinates using grid location and resolution.
    % [x0 y0] is the location of the start point in cell-units, relative to
    % the bottom left corner of the grid, gridLocation.
    x0 = (p1(1,1) - gridLocation(1,1))*resolution;
    y0 = (p1(1,2) - gridLocation(1,2))*resolution;
    
    for r = 1:numRays
        
        x = floor(x0) + 1;
        y = floor(y0) + 1;

        % [x1 y1] is the location of the rth end point in cell-units, relative to
        % the bottom left corner of the grid, gridLocation.
        x1 = (p2(r,1) - gridLocation(1,1))*resolution;
        y1 = (p2(r,2) - gridLocation(1,2))*resolution;
        
        % Get the number of increments and intersection points for ray
        m = 0;
        [dtx, xInc, txNext, l] = getIncrement(x0, x1, m);
        [dty, yInc, tyNext, n] = getIncrement(y0, y1, l);
        
        if n ~= 0
            % Reset vector with nans
            xp = nan(maxCells,1);
            yp = nan(maxCells,1);

            xp(1) = x;
            yp(1) = y;
            
            % The DDA algorithm works by coverting the slope of the ray to
            % the number of cells traveled along each axis, normalized using
            % the minor axis. For example, if the ray is formulated as y = 2x, 
            % with resolution 1, x is the minor axis, and moves 1 cell per step,
            % whereas y moves 2 cells for every step along the x axis.
            % In each iteration, the algorithm compares the distance to
            % the next gridline (in cells) along each axis, and
            % increments/updates the axis with larger distance until the
            % maximum number of iterations has been reached.
            % NOTE: One extension implemented in this algorithm is the handling
            % of cases where visited points along the ray [x,y] fall within
            % a certain tolerance of a cell boundary. In such cases, 
            % adjacent/overlapping cells are also included in the list of
            % midPoints and/or endPoints. This helps to prevent rays from
            % punching through obstacles if they pass through corners or along
            % cell boundaries.
            i = 1;
            iter = 1;
            while iter <= n
                if abs(tyNext - txNext) < 1e-15
                    % If corner point, then increment both
                    x = x + xInc;
                    txNext = txNext + dtx;

                    y = y +  yInc;
                    tyNext = tyNext + dty;

                    xp(i+1,1) = x;
                    yp(i+1,1) = y - yInc;

                    xp(i+2,1) = x - xInc;
                    yp(i+2,1) = y;

                    xp(i+3,1) = x;
                    yp(i+3,1) = y;
                    i = i + 3;
                    iter = iter + 2;
                    continue;
                end

                % Increment in X or Y direction
                if (tyNext < txNext)
                    % Increment y
                    y = y +  yInc;
                    tyNext = tyNext + dty;
                    xp(i+1,1) = x;
                    yp(i+1,1) = y;
                    i = i + 1;
                    if txNext > 1e10 && abs(round(x0) - x0) <= eps(x0)
                        % Point lies on vertical grid-line, include left and
                        % right cells
                        xp(i+1,1) = x + xInc;
                        yp(i+1,1) = y;
                        i = i + 1;
                    end
                elseif (tyNext > txNext)
                    % Increment x
                    x = x + xInc;
                    txNext = txNext + dtx;
                    xp(i+1,1) = x;
                    yp(i+1,1) = y;
                    i = i + 1;
                    if tyNext > 1e10 && abs(round(y0) - y0) <= eps(y0)
                        % Point lies on horizontal grid-line, include top and 
                        % bottom cells
                        xp(i+1,1) = x;
                        yp(i+1,1) = y + yInc;
                        i = i + 1;
                    end
                end

                iter = iter + 1;
            end
            
            % Remove NaN from the pre-allocated matrix
            idx = ~(isnan(xp(:)) | isnan(yp(:)));
            xpTemp = xp(idx);
            ypTemp = yp(idx);
            
            % Take last point as the endPt, check for case where end-point lies
            % near cell boundary
            [endPtX, endPtY] = handleEdgeConditions(x1, y1);
            if isempty(endPtX)
                endPtX = xpTemp(end);
                endPtY = ypTemp(end);
                xpTemp(end) = [];
                ypTemp(end) = [];
            else
                % Start and end point are in the same cell, just overwrite
                % first element of xp/yp
                xp(1) = x;
                yp(1) = y;
                % Check the last N pts in midPoints for endPt matches, remove if
                % found
                lastStartIdx = max(numel(xpTemp)-numel(endPtX),1);
                remCell = false(size(xpTemp));
                for i = lastStartIdx:numel(xpTemp)
                    remCell(i) = any((xpTemp(i) == endPtX) & (ypTemp(i) == endPtY)); % Point already in midPts
                end
                xpTemp(remCell) = [];
                ypTemp(remCell) = [];
            end
            
            % Handle edge cases for the starting point
            [xStart, yStart] = handleEdgeConditions(x0, y0);
            
            if ~isempty(xStart)
                lastStartIdx = min(numel(xpTemp),4);
                
                % Add edge case cells that aren't in endPts and aren't already in the midPt list
                remCell = false(size(xStart));
                for i = 1:numel(xStart)
                    remCell(i) = any((xStart(i) == endPtX) & (yStart(i) == endPtY)) || ... % Point also found in endPoints
                        any((xStart(i) == xpTemp(1:lastStartIdx) & yStart(i) == ypTemp(1:lastStartIdx))); % Point already in midPts
                end
                xpTemp = [xpTemp; xStart(~remCell)];
                ypTemp = [ypTemp; yStart(~remCell)];
            end
        else
            % Special case where start and end point are in the same cell.
            % Any cells touched by end point are endPts, and midPoints are
            % empty
            [endPtX, endPtY] = handleEdgeConditions(x1, y1);
            
            if isempty(endPtX)
                endPtX = x;
                endPtY = y;
            end
            
            xpTemp = [];
            ypTemp = [];
        end
        
        % Remove points outside of grid dimension.
        idx = xpTemp(:) >= 1 & xpTemp(:) <= gridSize(1,1) & ypTemp(:) >= 1 & ypTemp(:) <= gridSize(1,2);
        endidx = endPtX(:) >= 1 & endPtX(:) <= gridSize(1,1) & endPtY(:) >= 1 & endPtY(:) <= gridSize(1,2);
        
        if nargin == 5
            % Check all points for occupancy
            
            % Transform Y-axis as algorithm assumes grid origin is at bottom left while
            % matrix uses top-left origin.
            xpNew = [xpTemp(idx); endPtX(endidx)];
            ypNew = gridSize(1,2) + 1 - [ypTemp(idx); endPtY(endidx)];
            
            rayIndices = ypNew + (xpNew-1)*size(map,1);
            cellIsOccupied = map(rayIndices);
            occupiedCellIndex = find(cellIsOccupied, 1, 'first');
            if ~isempty(occupiedCellIndex)
                % collision
                isCollisionFree(r) = false;
                collisionGridPt(r,:) = reshape([ypNew(occupiedCellIndex) xpNew(occupiedCellIndex)],1,2);
            end
        else
            % Transform Y-axis as algorithm assumes grid origin is at bottom left while
            % matrix uses top-left origin.
            xpNew = xpTemp(idx);
            ypNew = gridSize(1,2) + 1 - ypTemp(idx);
            
            % If occupancy grid is not supplied then return without checking for collision
            endPts = [gridSize(1,2) + 1 - endPtY(endidx), endPtX(endidx)];
            ePts = size(endPts,1);
            mPts = numel(xpNew);
            if ~rangeIsMax(r)
                % endPts are valid obstacles, add them to endPtsVec
                if ePts > 0
                    endPtsVec(curEndIdx:curEndIdx+ePts-1,:) = endPts;
                    curEndIdx = curEndIdx + ePts;
                end
            end
            % Add remaining points to midPtsVec
            if mPts > 0
                midPtsVec(curMidIdx:curMidIdx+mPts-1,:) = [ypNew xpNew];
                curMidIdx = curMidIdx + mPts;
            end
        end
    end
    midPtsVec(curMidIdx:end,:) = [];
    endPtsVec(curEndIdx:end,:) = [];
end

function [dtx, xInc, txNext, n] = getIncrement(x0, x1, n)
%getIncrement Get the X-Y increments and intersection points
%   Compute various algorithm parameters based on the difference between
%   the coordinates of the start and end point.

    coder.inline('always')
    % Sign of dx represents the direction of the line
    dx = (x1 - x0);

    % For horizontal or vertical lines, if the point is eps away from the edge,
    % then make sure that xInc is not zero. The sign of xInc is decided based
    % on which side of edge the point lies.
    xIncSign = -1;
    if round(x0) - floor(x0) > 0
        xIncSign = 1;
    end

    % Intersection of the line with the circle of unit radius used to compute
    % intersecting points with lines (algorithm parameter)
    dtx = 1.0/abs(dx);

    % Compute how to increment the X and Y grid coordinates
    if abs(dx) <= 2*eps(x1)
        xInc = xIncSign;
        txNext = dtx;
    elseif dx > 0
        xInc = 1;
        n = n + floor(x1) - floor(x0);
        txNext = (floor(x0)+ 1 - x0) * dtx;
    else
        xInc = -1;
        n = n + floor(x0) - floor(x1);
        txNext = (x0 - floor(x0)) * dtx;
    end
end

function [xp, yp] = handleEdgeConditions(x, y)
%handleEdgeConditions Handle start and end point edge conditions
%   The edge condition is that the start and end points can be on a corner
%   of the cell or on the edge of the cell. Based on this fact, consider
%   nearby cells in the collision check. Inputs x and y are real-values 
%   scalars, where [x,y] defines either the start or end point of the ray,
%   converted to cells, relative to the bottom-left corner of the grid. The
%   outputs, xp and yp, are N-by-1 vectors of integers corresponding to the
%   locations of cells touched by [x,y], relative to the bottom-left corner
%   of the grid.
    coder.inline('always')
    if abs(x - floor(x)) <= 2*eps(x)
        % If start/end point is on a corner
        if abs(y - floor(y)) <= 2*eps(y)
            xp = floor(x) + [0; 1; 0; 1];
            yp = floor(y) + [1; 0; 0; 1];
        elseif abs(y - ceil(y)) <= 2*eps(y)
            xp = floor(x) + [0; 1; 0; 1];
            yp = ceil(y) + [1; 0; 0; 1];
        else
            xp = floor(x);
            yp = floor(y)+1;
        end
    elseif abs(x - ceil(x)) <= 2*eps(x)
        % If start/end point is on a corner
        if abs(y - floor(y)) <= eps(y)
            xp = ceil(x)  + [0; 1; 0; 1];
            yp = floor(y) + [1; 0; 0; 1];
        elseif abs(y - ceil(y)) <= 2*eps(y)
            xp = ceil(x) + [0; 1; 0; 1];
            yp = ceil(y) + [1; 0; 0; 1];
        else
            xp = ceil(x);
            yp = floor(y)+1;
        end
    elseif abs(y - floor(y)) <= 2*eps(y)
        % If start/end point is on an edge
        xp = floor(x)+1;
        yp = floor(y);
    elseif abs(y - ceil(y)) <= 2*eps(y)
        % If start/end point is on an edge
        xp = floor(x)+1;
        yp = ceil(y)+1;
    else
        xp = [];
        yp = [];
    end
end