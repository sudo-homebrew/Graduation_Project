function [assignments,cost,gap] = assignkbestsd(varargin)
%ASSIGNKBESTSD    K-best solutions to the S-D assignment problem
%   [assignments, cost, gap] = ASSIGNKBESTSD(costMatrix) 
%   finds the best S-D solution that minimize the total cost of assignment.
%   The algorithm uses Lagrangian relaxation to convert the S-D assignment
%   problem to a corresponding 2-D assignment problem and then solves the
%   2-D problem.
%
%   [...] = ASSIGNKBESTSD(costMatrix, k) finds the k-best S-D assignment
%   solutions that minimize the total cost of assignment. The algorithm
%   uses Murty's algorithm to find k-1 solutions, in addition to the best
%   solution. If not specified, the default value for k is 1.
%
%   [...] = ASSIGNKBESTSD(..., k, desiredGap), additionally, lets you
%   specify the desired maximum gap between the dual solution and the
%   feasible solution as a scalar usually ranging between 0 and 1. A value
%   of 0 means the dual and feasible solutions are the same. The gap can be
%   used to control the quality of the solution. If not specified, the
%   default value for the desiredGap is 0.01.
%
%   [...] = ASSIGNKBESTSD(..., k, desiredGap, maxIterations), additionally,
%   lets you specify the maximum number of iterations the assignsd is
%   allowed. If not specified, the default value for maxIterations is 100.
%   desiredGap and maxIterations define the terminating conditions for the
%   algorithm.
%
%   [...] = ASSIGNKBESTSD(..., k, desiredGap, maxIterations, algorithm),
%   additionally, lets you specify the algorithm to use for solving the 2-D
%   assignment problem. Allowed options are 'munkres', 'auction' and 'jv'.
%   If not specified, the default value for algorithm is 'auction'.
%
%   costMatrix is an n-dimensional cost matrix where costMatrix(i,j,k ...)
%   defines the cost of the n-tuple (i,j,k, ...) in assignment. The index
%   '1' on all dimensions in the costMatrix represents dummy measurement or
%   a false track and is used to complete the assignment problem. The index
%   1, being a dummy, can be a part of multiple n-tuples, i.e., it is not
%   constrained to be assigned only once. A typical cost value for
%   costMatrix(1,1,1,1,..) is 0.
%
%   assignments is a k-element cell array. Each cell element is a P-by-N
%   list of assignment returned by the S-D assignment function, assignsd. 
%   Assignments of the type [1 1 Q 1] from a 4-dimensional cost matrix can
%   be seen as Q-1 entity from dimension 3 was left unassigned. The cost
%   value at (1,1,Q,1) defines the cost of not assigning the (Q-1)th entity
%   from dimension 3.
%
%   costOfAssignment is a k-element array. The j-th element is a scalar
%   value summarizing the total cost of the j-th solution to the S-D
%   assignment.
%
%   solutionGap is a positive k-element array. Each element is the duality
%   gap achieved between the feasible and dual solution. A value of
%   solutionGap near zero indicates the quality of solution.
%
%   Example:
%   --------
%   % Find the first 5 best assignments of the S-D assignment problem
%   % Load the cost matrix
%   load passiveAssociationCostMatrix.mat
%   [assignments, cost, gap] = ASSIGNKBESTSD(costMatrix,5,0.01,100)
%
%   See also: assignsd, assignkbest
%
%   References:
%   [1] Popp, R. L., Pattipati, K., and Bar-Shalom, Y. (2001). M-best S-D
%       Assignment Algorithm with Application to Multitarget Tracking. IEEE
%       Transactions on Aerospace and Electronic Systems, 37(1), 22-39.
%   [2] Deb, S., Yeddanapudi, M., Pattipati, K., & Bar-Shalom, Y. (1997).
%       A generalized SD assignment algorithm for multisensor-multitarget
%       state estimation. IEEE Transactions on Aerospace and Electronic
%       Systems, 33(2), 523-538.

%   Copyright 2018 The MathWorks, Inc.

%#codegen

narginchk(1,5);

[costMatrix, numSolutions, desiredGap, maxIterations, algorithm] = parseAndValidateInputs(varargin{:});
costDim = ndims(costMatrix); 

[assignment,c,g] = fusion.internal.assignment.kbestGetSDSolution(costMatrix,desiredGap,maxIterations,algorithm);

% Format of solutionsList
% 1st element: constrained cost matrix
% 2nd element: solution to constrained cost matrix
% 3rd element: [cost, gap of assignment]
% 4th element: number of assignments in solutions which were enforced.
formatSolutionList = {{costMatrix,assignment,[c,g],0}};
solutionList = repmat(formatSolutionList,[1 1]);
coder.varsize('solutionList',[1 inf],[0 1]);
solutionList{1}{1} = costMatrix;
solutionList{1}{2} = assignment;
solutionList{1}{3} = [c, g];
solutionList{1}{4} = 0;
optimalSolutions = repmat(solutionList,[numSolutions 1]);

% Sweep N times to obtain the N-best solutions
currentSweep = 1;

while ~isempty(solutionList) && currentSweep <= numSolutions
    % Pick up best solution from Queue. It's at the top.
    % Partition the solution.
    tempSolutionList = fusion.internal.assignment.kbestSDPartitionSolution({solutionList{1}}, desiredGap, maxIterations, algorithm);
    coder.varsize('tempSolutionList',[1 inf],[0 1]);
    
    % Add new solutions to the queue.
    if coder.target('MATLAB')
        if ~isempty(tempSolutionList)
            solutionList = [solutionList tempSolutionList]; %#ok<AGROW>
        end
    else
        for i = 1:numel(tempSolutionList)
            solutionList{end+1} = tempSolutionList{i};
        end
    end
    
    % The solutionList may also contain assignments which are already
    % partitioned and stored in optimal list. They should not be
    % partitioned again. The solutionList may itself also contain duplicate
    % partitions. The following code should remove both of these
    % duplicates.
    if coder.target('MATLAB')
        checkSolutionList =  [optimalSolutions(1:currentSweep-1)' solutionList];
        removedDupsList = fusion.internal.assignment.kbestRemoveDuplicates(checkSolutionList,size(costMatrix));
        % The last solutions are what we need as the first currentSweep-1 had
        % no duplicates.
        solutionList = {removedDupsList{currentSweep:end}};
    else
        checkSolutionList = cell(1,0);
        for iterSweep = 1:currentSweep - 1
            checkSolutionList{end+1} = optimalSolutions{iterSweep};
        end
        for solSweep = 1:numel(solutionList)
            checkSolutionList{end+1} = solutionList{solSweep};
        end
        removedDupsList = fusion.internal.assignment.kbestRemoveDuplicates(checkSolutionList,size(costMatrix));
        newList = repmat(solutionList,[1 0]);
        for dupSweep = currentSweep:numel(removedDupsList)
            newList{end+1} = removedDupsList{dupSweep};
        end
        solutionList = newList;
    end
    
    % Rank the solutions in increasing order of cost
    solutionList = fusion.internal.assignment.kbestRankSolutions(solutionList);
    
    optimalSolutions{currentSweep}{1} = solutionList{1}{1};
    optimalSolutions{currentSweep}{2} = solutionList{1}{2};
    optimalSolutions{currentSweep}{3} = solutionList{1}{3};
    optimalSolutions{currentSweep}{4} = solutionList{1}{4};
    
    % Update sweep
    currentSweep = currentSweep + 1;
    
    if coder.target('MATLAB')
        solutionList(1) = [];
    else
        newList = repmat(solutionList,[1 0]);
        for i = 2:numel(solutionList)
            newList{end+1} = solutionList{i};
        end
        solutionList = newList;
    end
end

% Number of solutions found might be less than number of solutions
% requested. 
numSolnFound = currentSweep - 1;

if coder.target('MATLAB')
    foundSolutions = fusion.internal.assignment.kbestRankSolutions(optimalSolutions(1:numSolnFound));
else
    foundSolutions = repmat(solutionList,[1 0]);
    for i = 1:numSolnFound
        foundSolutions{end+1} = optimalSolutions{i};
    end
    foundSolutions = fusion.internal.assignment.kbestRankSolutions(foundSolutions);
end

% Allocate memory for assignment
assignments    = repmat({zeros(0,costDim, 'uint32')}, [numSolnFound, 1]);
cost = inf(numSolnFound,1,'like',costMatrix);
gap = zeros(numSolnFound,1,'like',desiredGap);

for i = 1:numSolnFound
    assignments{i} = sortrows(foundSolutions{i}{2});
    cost(i) = foundSolutions{i}{3}(1);
    gap(i) = foundSolutions{i}{3}(2);
end

end

%% Input Parsing Functions
function [costMatrix, numSolutions, desiredGap, maxIterations, algorithm] = parseAndValidateInputs(varargin)
% Parses and validates the inputs to the main function

% validate costMatrix
funcName = mfilename;
validateattributes(varargin{1},{'double','single'},{'real','nonsparse','nonnan','nonempty'},funcName,'Cost Matrix',1);
costMatrix = varargin{1};
classToUse = class(costMatrix);
numDims = numel(size(costMatrix));
coder.internal.errorIf(numDims<3, 'fusion:assignsd:expectedAtleast3D');

% Validate k
if nargin > 1
    validateattributes(varargin{2}, {'numeric'}, ...
        {'nonsparse','real','finite','scalar','integer','positive'}, mfilename, 'k',2);
    numSolutions = varargin{2};
else
    numSolutions = 1;
end

% Validate desired gap
if nargin > 2
    validateattributes(varargin{3},{classToUse},...
        {'scalar', 'finite', 'real', 'nonsparse','positive','<=',1},funcName,'Desired Gap',3);
    desiredGap = varargin{3};
else
    desiredGap = cast(0.01,classToUse);
end

% Validate max number of iterations
if nargin > 3
    validateattributes(varargin{4}, {'numeric'}, ...
        {'nonsparse','real','finite','scalar','integer','positive'}, funcName, 'Max Iterations',4);
    maxIterations = varargin{4};
else
    maxIterations = 100;
end

validAlgs = {'munkres', 'jv', 'auction'};
if nargin>4
    % Find the function to use in the list of all valid algorithms
    algorithm = validatestring(varargin{5}, validAlgs, mfilename, 'Algorithm', 5);
else
    algorithm = 'auction';
end
end
