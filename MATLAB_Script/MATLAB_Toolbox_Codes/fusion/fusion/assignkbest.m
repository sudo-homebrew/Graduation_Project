function [assignments,unassignedRows,unassignedCols,cost] = assignkbest(varargin)
%ASSIGNKBEST K-best solutions to the assignment problem
%   [assignments, unassignedTracks, unassignedDetections, cost] = ...
%       ASSIGNKBEST(costMatrix, costOfNonAssignment) finds the best
%   global nearest-neighbor solution that minimize the total cost of
%   assignment. It uses the Jonker-Volgenant algorithm to minimize the cost
%   of assignment.
%
%   [...] = ASSIGNKBEST(costMatrix, costOfNonAssignment, k) finds the
%   K-best global nearest-neighbor solutions that minimize the total cost
%   of assignment. It uses Murty's algorithm to find K-1 solutions, in
%   addition to the best solution.
%
%   [...] = ASSIGNKBEST(..., k, algorithm), additionally, lets you specify
%   an algorithm used for finding the assignment as a character array or a
%   scalar string The choices of algorithm are: 'munkres' for the Hungarian
%   algorithm, 'jv' for the Jonker-Volgenant algorithm, 'auction' for
%   Auction algorithm and 'matchpairs' for the algorithm implemented by
%   matchpairs function. By default, 'algorithm' is 'jv'.
%
%   costMatrix is an M-by-N matrix, where M is the number of tracks, and N
%   is the number of detections. costMatrix(i,j) is the cost of assigning
%   j-th detection to i-th track. The lower the cost, the more likely the
%   assignment is to be made.
%
%   costOfNonAssignment is a either:
%
%   A scalar, which represents the cost of a track or a detection remaining
%   unassigned. Higher costOfNonAssignment corresponds to the higher
%   likelihood that every existing track will be assigned a detection.
%
%   A 2-element vector, where the first element defines the cost of a
%   detection remaining unassigned and the second element defines the cost
%   of a track remaining unassigned.
%
%   A cell array of vectors. The first element of the cell array has
%   a length equal to the number of detections and defines the cost of
%   detections remaining unassigned. The second element of the cell array
%   has a length equal to the number of tracks and defines the cost of
%   tracks remaining unassigned.
%
%   assignments is a k-element cell array. Each cell element is an Li-by-2
%   matrix of index pairs of tracks and corresponding detections, where Li
%   is the number of pairs in the ith solution. The first column contains
%   the row indices and the second column contains the corresponding
%   column indices.
%
%   unassignedRows is a k-element cell array. Each cell element is a
%   Pi-element, where Pi = M - Li is the number of unassigned rows. Each
%   element is an index of a row to which no columns were assigned.
%
%   unassignedCols is a k-element cell array. Each cell element is a
%   Qi-element vector, where Qi = N - Li is the number of unassigned
%   columns. Each element is an index of a column that was not assigned to
%   any rows.
%
%   cost is a k-element array. Each element is a scalar value summarizing
%   the total cost of the solution to the assignment problem.
%
%   Example:
%   --------
%   % Find the first 5 best assignments (example taken from [1]).
%   x = inf; % This indicates an invalid assignment.
%   costMatrix = [10 5 8 9; 7 x 20 x; x 21 x x; x 15 17 x; x x 16 22];
%   costOfNonAssignment = 100;
%   [assignments, unassignedRows, unassignedCols, cost] = ASSIGNKBEST(costMatrix, costOfNonAssignment,5)
%
%   See also: assignmunkres, assignjv, assignauction
%
%   References:
%
%   [1] Samuel Blackman and Robert Popoli, "Design and Analysis of Modern
%       Tracking Systems", Artech House, 1999.

%   Copyright 2017-2020 The MathWorks, Inc.

%#codegen

[costMatrix, costOfNonAssignment, k, algorithm] = fusion.internal.assignment.parseAssignkbestInputs(varargin{:});

% Create the Problem/solution pair
switch algorithm
    case 'munkres'
        ps = fusion.internal.assignment.MunkresProblemSolutionPair(costMatrix, costOfNonAssignment);
    case 'matchpairs'
        ps = fusion.internal.assignment.MatchPairsProblemSolutionPair(costMatrix, costOfNonAssignment);
    case 'auction'
        ps = fusion.internal.assignment.AuctionProblemSolutionPair(costMatrix, costOfNonAssignment);
    case 'jv'
        ps = fusion.internal.assignment.JVProblemSolutionPair(costMatrix, costOfNonAssignment);
    otherwise
        assert(false);
end

% Initialize priority queue
[nRow, nCol] = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntSize2D(costMatrix);
N = nRow + nCol;
maxNumProblems = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntIndex(N*k + 1);
currentIteration = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntZero();

priorityQueue = fusion.internal.assignment.ProblemSolutionQueue(ps,maxNumProblems);
priorityQueue = addProblems(priorityQueue, {ps});

% Output solution Queue
solutionQueue = fusion.internal.assignment.ProblemSolutionQueue(ps,k);

while ~isDone(priorityQueue) && currentIteration < k
    % Extract the best solution from the queue
    [priorityQueue, bestSolution, isValidSoln] = extractBestSolution(priorityQueue);
    
    % If the best solution is empty or invalid, exit the while loop.
    if ~isValidSoln
        break;
    end
    
    % Take the best solution and add it to the list of output solutions
    solutionQueue = addProblems(solutionQueue, {bestSolution});
    
    % Partition the best solution
    partitionSolutions = partition(bestSolution);
    
    % Add the new solutions to priority queue and sort it
    priorityQueue = addProblems(priorityQueue, partitionSolutions);
    priorityQueue = sortQueue(priorityQueue);
    
    % Move to next step
    currentIteration = currentIteration + 1;
end

% Format output
[assignments, unassignedRows, unassignedCols, cost] = formatSolution(solutionQueue);

end
