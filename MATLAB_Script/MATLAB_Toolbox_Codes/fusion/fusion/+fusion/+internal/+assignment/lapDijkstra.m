function [rowSoln, colSoln, colRedux] = ...
    lapDijkstra(costMatrix, rowSoln, colSoln, colRedux)
% This function is for internal use only. It may be removed in the future.

%LAPDIJKSTRA Solution to the Linear Assignment Problem (LAP) using Dijkstra's shortest path algorithm
%   [ROWSOLN, COLSOLN, COLREDUX] = ...
%     LAPDIJKSTRA(COSTMATRIX, ROWSOLN, COLSOLN, COLREDUX)
%   Assigns rows to columns based on the COSTMATRIX using Dijkstra's
%   shortest path assignment algorithm, where each column is assigned to a
%   row in a way that minimizes the total cost.
%
%   COSTMATRIX is an M-by-N matrix, where each element defines the cost of
%   assigning column n to row m is represented as COSTMATRIX(m,n). Larger
%   assignment costs mean that the assignment is less likely to selected by
%   the algorithm as it seeks to minimize the overall cost of assignment of
%   all columns to rows.
%
%   ROWSOLN is an M element column vector, with each element set to the
%   column assigned to the corresponding row. When no column is assigned,
%   the element is set to NaN.
%
%   COLSOLN is an N element row vector, with each element set to the row
%   assigned to the corresponding column. When no row is assigned, the
%   element is set to NaN.
%
%   COLREDUX is an N element row vector of the column reduction values
%   corresponding to the returned assignment solution.
%
%   Algorithm can use a partial solution to the LAP by passing in the
%   ROWSOLN, COLSOLN, and COLREDUX from a previous partial solution. If not
%   provided, the algorithm assumes no partial solution has been provided.

% Copyright 2017 The MathWorks, Inc.
%
%   Reference:
%   ----------
%   [1] E. Dijkstra, A note on two problems in connexion with graphs,
%   Numerische Mathematik 1, 1959, 269-271.

%#codegen

% Augmentation is the final phase of JV assignment. Here, a modified
% version of Dijkstra's shortest path algorithm is used to assign the
% remaining rows of the cost matrix.

% Define local variables required for integer arithmetic in generated code.

ZERO = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntZero();
ONE = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();
[nRow, nCol] = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntSize2D(costMatrix);
N1 = ZERO;
N2 = ZERO;
rowOld = ONE;

if nargin < 2 || isempty(rowSoln)
    rowSoln = NaN(nRow, 1, 'like', costMatrix);
end

if nargin < 3 || isempty(colSoln)
    colSoln = NaN(1, nCol, 'like', costMatrix);
end

if nargin < 4 || isempty(colRedux)
    colRedux = zeros(1, nCol, 'like', costMatrix);
end

% Nothing to do?
isnanRowSoln = isnan(rowSoln);
if ~any(isnanRowSoln)
    return;
end

N2(1) = sum(isnanRowSoln);
assert(N2 <= nRow); % Assist MATLAB Coder
freeRows = coder.nullcopy(repmat(ONE,N2,1));
freeRows(:) = find(isnan(rowSoln));
nFree = N2;

for thisRow = ONE:nFree
    
    rowFree = freeRows(thisRow);
    
    dist = costMatrix(rowFree,:) - colRedux; % cost of assigning this row to each col
    
    % Do any valid distances exist for this row?
    if ~all(isinf(dist))
        
        % Paths alternate between rows and cols, prevRow keeps track of each
        % column's previous row
        prevRow = rowFree*repmat(ONE,ONE,nCol); % Set previous row in tree as this row
        
        % colList places columns in 1 of 3 queues. 'low' and 'up' partition
        % colList according to the table below. Its entries are indices into
        % colSoln
        %   [1:low-1]   : columns ready to have their assignment finalized
        %   [low:up-1]  : columns to be scanned for *current* minimum
        %   [up:n]      : columns to be scanned later for a *new* minimum
        low = ONE;
        up = ONE;
        colList = ONE:nCol;
        last = ZERO;
        endOfPath = ZERO;
        
        unassignedFound = false;
        noMinFound = false;
        distMin = zeros(1,'like',dist);
        while ~unassignedFound
            % If no min-valued columns exist, search for a new set
            if low == up
                last = low-ONE;
                
                % Find all columns with min value and place them in the
                % 'current minimum' queue
                colListK = colList(up:nCol);
                distColListK = dist(colListK);
                N1(1) = numel(colListK);
                assert(N1 <= nCol); % Assist MATLAB Coder
                distMin = min(distColListK(1,1:N1));
                
                % Skip to the next row since there are no min-valued
                % columns left for this row
                if isinf(distMin)
                    noMinFound = true;
                    break;
                end
                
                isMin = dist(colListK) == distMin;
                colListKMin = colListK(isMin);
                colListKNotMin = colListK(~isMin);
                
                % Check if any of the min-valued columns correspond to an
                % unassigned column, if so the alternating path is complete
                N1(1) = numel(colListKMin);
                assert(N1 <= nCol); % Assist MATLAB Coder
                tmpVar = colSoln(colListKMin);
                isUnassignedCol = isnan(tmpVar(1,1:N1));
                if any(isUnassignedCol)
                    endOfPath = colListKMin(isUnassignedCol);
                    endOfPath = endOfPath(1);
                    unassignedFound = true;
                else
                    % Swap values to place in queue and avoid overwrite
                    colList(up:nCol) = [colListKMin colListKNotMin];
                    N1(1) = numel(isMin);
                    assert(N1 <= nCol); % Assist MATLAB Coder
                    N1(1) = sum(isMin(1,1:N1));
                    up = up + N1;
                end
            end
            
            if ~unassignedFound
                
                % Use a modified version of Dijkstra's algorithm to find the
                % path alternating between rows and columns with the least
                % total cost of assignment
                %
                % This maps into Dijkstra's algorithm as follows:
                %
                % - The source vertex is the new assignment: (rowFree, colMin)
                % - The distance of the source is its cost
                % - The neighboring vertices are the assignments of the
                %   currently assigned row (rowOld) with all of the
                %   non-min-valued columns (newCol)
                % - The distance of each neighbor is its cost
                % - The length of the edge connecting a neighbor to the source
                %   is the difference in cost between rowOld and rowFree
                %
                % The path terminates when an unassigned column is found
                
                colMin = colList(low);
                low = low+ONE; % Search next min-valued column on next loop
                
                rowOld(1) = colSoln(colMin); % Currently assigned row
                costOld = costMatrix(rowOld, colMin) - colRedux(colMin);
                
                colNew = colList(up:nCol);
                costNew = costMatrix(rowOld, colNew) - colRedux(colNew);
                
                % Compute edge lengths
                edgeLen = costNew - costOld;
                
                % Apply relaxation
                distAdj = edgeLen + distMin;
                doRelax = distAdj < dist(colNew);
                dist(colNew(doRelax)) = distAdj(doRelax);
                prevRow(colNew(doRelax)) = rowOld;
                
                % Check for terminal condition for the alternating path for any
                % of the relaxed columns
                isMinLen = (distAdj == distMin) & doRelax;
                isTerminal = isnan(colSoln(colNew)) & isMinLen;
                if any(isTerminal)
                    endOfPath = colNew(find(isTerminal, 1));
                    unassignedFound = true;
                elseif any(isMinLen)
                    % Terminal condition not met, add relaxed columns to be
                    % scanned on next loop
                    
                    % Swap values to place in queue to avoid overwrite
                    N1(1) = numel(isMinLen);
                    assert(N1 <= nCol); % Assist MATLAB Coder
                    N1(1) = sum(isMinLen(1,1:N1));
                    
                    tmp = colNew;
                    colList(up:nCol) = [tmp(isMinLen) tmp(~isMinLen)];
                    up = up+N1;
                end
            end
        end
        
        if ~noMinFound
            % Update column reduction values based on new assignment
            colReady = colList(1:last);
            colRedux(colReady) = colRedux(colReady) + dist(colReady) - distMin;
            
            % Set row and column assignments based on shortest path starting at the
            % end of the path and marching backwards until rowFree is reached
            while true
                rowAssigned = prevRow(endOfPath);
                colAssigned = endOfPath;
                
                colSoln(endOfPath) = rowAssigned;
                endOfPath(1) = rowSoln(rowAssigned);
                rowSoln(rowAssigned) = colAssigned;
                
                if rowAssigned == rowFree
                    break;
                end
            end
        end
    end
end
end
