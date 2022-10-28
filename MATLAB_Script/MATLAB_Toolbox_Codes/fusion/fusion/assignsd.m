function [assignments,costOfAssignment,solutionGap] = assignsd(costMatrix,varargin)
% ASSIGNSD S-D assignment using Lagrangian relaxation
%   [assignments, costOfAssignment, solutionGap] = ASSIGNSD(costMatrix)
%   computes a sub-optimal solution to the S-D assignment problem using
%   Lagrangian relaxation method. The algorithm terminates when the gap
%   reaches below 0.01 (1 percent) or if the number of iterations reaches
%   100. 
%
%   [...] = ASSIGNSD(costMatrix,desiredGap) allows specifying the minimum 
%   gap desired.
%
%   [...] = ASSIGNSD(costMatrix,desiredGap,maxIterations) allows specifying 
%   the maximum number of allowed iterations.
%
%   [...]= ASSIGNSD(costMatrix,desiredGap, maxIterations, algorithm) allows
%   specifying the algorithm to use for solving the 2-D assignment problem.
%   Allowed options are 'munkres', 'auction' and 'jv'. Default algorithm is
%   'auction'.
%
%   The Lagrangian relaxation method computes a sub-optimal solution to
%   the S-D assignment problem. The method relaxes the S-D assignment
%   problem to a 2-D assignment problem using a set of Lagrangian
%   multipliers. The relaxed 2-D assignment problem is commonly known as
%   the dual problem, which can be solved optimally using algorithms like
%   Munkres Algorithm. Constraints are then enforced on the dual solution,
%   by solving multiple 2-D assignment problems, to obtain a feasible 
%   solution to the original problem. The cost of dual solution and 
%   feasible solution serves as lower and upper bound on the optimal cost 
%   respectively. The algorithm iteratively tries to minimize the gap 
%   between dual and feasible solution, commonly known as the dual gap, 
%   till it reaches below a certain desired gap or returns the current best 
%   solution if the maximum number of iterations have reached.
%
%   costMatrix is a n-dimensional cost matrix where costMatrix(i,j,k ...)
%   defines the cost of the n-tuple (i,j,k, ...) in assignment. The index
%   '1' on all dimensions in the costMatrix represents dummy  measurement
%   or a false track and is used to complete the assignment problem. The
%   index 1, being a dummy, can be a part of multiple n-tuples i.e. it is
%   not constrained to be assigned only once. A typical cost value for
%   costMatrix(1,1,1,1,..) is 0.
%
%   desiredGap defines the desired normalized relative gap between dual
%   solution and feasible solution. It is scalar value usually ranging
%   between 0 and 1. A value of 0 means dual and feasible solution are
%   same. It can be used to control the quality of solution.
%
%   maxIterations defines the maximum number of iterations allowed for
%   solving the assignment algorithm.
%
%   desiredGap and maxIterations define the terminating conditions for the
%   algorithm.
%
%   assignments is a P-by-N list of assignment returned by the algorithm.
%   assignments of the type [1 1 Q 1] from a 4-dimensional cost matrix can
%   be seen as Q-1 entity from dimension 3 was left unassigned. The cost
%   value at (1,1,Q,1) defines the cost of not assigning the (Q-1)th entity
%   from dimension 3.
%
%   costOfAssignment is a scalar value indicating the total cost of the
%   assignment.
%
%   solutionGap is a positive scalar value indicating the duality gap
%   achieved between the feasible and dual solution. A value of solutionGap
%   near zero indicates the quality of solution.
%
%   Notes:
%
%   1. The function uses the Heuristic Price Update to update the
%      Lagrangian multipliers, when using 'auction' algorithm. For 'munkres'
%      and 'jv' algorithms, the function uses the Accelerated Subgradient
%      Update.
%   2. For cost matrices with well-defined solutions, like the passive
%      association with high-precision sensors, the solutionGap
%      converges to within 0.05 (5 percent) in about 100 iterations.
%   3. As the optimal solution is unknown, the solutionGap may be non-zero
%      even when the returned solution is optimal.
%
%   % Example 1: Passive sensor data association (Pg. 415 from [2])
%   % ---------------------------------------------------------------------
%   % Load the cost matrix
%   load passiveAssociationCostMatrix.mat
%   [assignments,cost,gapAchieved] = assignsd(costMatrix,0.01,100)
%
%   % Example 2: Perform strict assignment without index 1.
%   % ---------------------------------------------------------------------
%   % No dummy index indicates that no entity can be left unassigned.
%   % Hence this needs to be an equi-dimensional cost matrix.
%   costMatrix = rand(6,6,6);
%   % Define a Inf full matrix with one dimension higher than cost matrix
%   fullMatrix = inf(7,7,7);
%   % Set the inner matrix to costMatrix to force the assignments
%   % including index 1 to have cost = Inf.
%   fullMatrix(2:end,2:end,2:end) = costMatrix;
%   fullMatrix(1,1,1) = 0;
%   [assignments,cost,gapAchieved] = assignsd(fullMatrix,0.01,100);
%   % Restore actual indices.
%   assignments = assignments - 1
%
%   See also: assignauction, assignjv, assignmunkres
%
%   References:
%   [1] Deb, S., Yeddanapudi, M., Pattipati, K., & Bar-Shalom, Y. (1997).
%       A generalized SD assignment algorithm for multisensor-multitarget
%       state estimation. IEEE Transactions on Aerospace and Electronic Systems
%       , 33(2), 523-538.
%   [2] Blackman, Samuel, and Robert Popoli. "Design and analysis of modern
%       tracking systems(Book)." Norwood, MA: Artech House, 1999. (1999).

%   Copyright 2018 The MathWorks, Inc.

%#codegen

% validate Inputs
narginchk(1,4)
[costMatrix,desiredGap,maxIterations,algFcn,priceUpdate] = parseAndValidateInputs(costMatrix,varargin{:});
totalDummyCost = costMatrix(1);
costMatrix(1) = 0;
classToUse = class(costMatrix);

% Assess size and type of the problem.
dimSize = size(costMatrix);

numDims = numel(dimSize);

% We can remove impossibles first so that they don't affect scaling and
% lagrangian multiplier updates.
costMatrix = fusion.internal.assignment.sdRemoveImpossibles(costMatrix);

% number of constraints = S - 2;
numConstraints = numDims - 2;

% Pre-allocation of variables.
% Lagrange Multiplier (n3x1, n4x1, n5x1 ...)
% Constrained Cost matrices (n1xn2, n1xn2xn3, n1xn2xn3xn4 ...)

% The following parameters vary in size for each constraint.
% Need to store them in a cell.
lagrangianMultipliers = cell(numConstraints,1);
constrainedCost = cell(numConstraints,1);
costIndices = cell(numConstraints,1);
gradients = cell(numConstraints,1);
p = cell(numConstraints,1);
hessianMats = cell(numConstraints,1);
feasibleCost = zeros(numConstraints,1,classToUse);
beta = ones(numConstraints,1,classToUse);
price = cell(numConstraints,1);
dummyAssignments = cell(numConstraints,1);

% Initialize variables
for i = 1:numConstraints
    currentDim = numDims - i + 1;
    lagrangianMultipliers{i} = zeros([ones(1,numDims-i) dimSize(currentDim)],classToUse);
    constrainedCost{i} = zeros(dimSize(1:numDims-i),classToUse);
    costIndices{i} = zeros(dimSize(1:numDims-i));
    gradients{i} = zeros(dimSize(currentDim),1,classToUse);
    hessianMats{i} = eye(dimSize(currentDim),classToUse);
    p{i} = zeros(dimSize(currentDim),1,classToUse);
    price{i} = zeros(dimSize(currentDim),1,classToUse);
    dummyAssignments{i} = zeros(dimSize(currentDim),1,classToUse);
end

% Initiate best dual cost so far
bestDualCost = cast(-Inf,classToUse);
bestFeasibleCost = Inf*ones(numConstraints,1,classToUse);
bestSolution = zeros(0,numDims,'uint32');

% Start while loop
currentIter = uint32(1);
currentGap = cast(Inf,classToUse);
bestGap = cast(Inf,classToUse);
coder.varsize('constrainedAssignment',[Inf numDims],[1 1]);
coder.varsize('feasibleCostMatrix');

while currentGap > desiredGap && currentIter-1 < maxIterations
    % Step 1: Relax constrains from cost to reach 2-D cost
    % -----------------------------------------------------
    coder.unroll();
    for i = 1:numConstraints
        if i == 1
            matrix = bsxfun(@minus,costMatrix,lagrangianMultipliers{i});
            [constrainedCost{i},costIndices{i}] = min(matrix,[],numDims-i+1);
        else
            matrix = bsxfun(@minus,constrainedCost{i-1},lagrangianMultipliers{i});
            [constrainedCost{i},costIndices{i}] = min(matrix,[],numDims-i+1);
        end
        % The first index is not a binary variable. Reference [1] Eq. (11)
        % is incorrect when all indices are dummy. For all dummy
        % assignments (index 1), pick the sum of all the negative values.
        if coder.target('MATLAB')
            numOnes = num2cell(ones(numDims-i,1));
        else
            numOnes = cell(numDims-i,1);
            for iterOnes = 1:numDims-i
                numOnes{iterOnes} = 1;
            end
        end
        allValues = zeros(dimSize(numDims-i+1),1,classToUse);
        allValues(:) = matrix(numOnes{:},:);
        negIndices = find(allValues < 0);
        negValues = allValues(negIndices);
        dummyAssignments{i}(:) = 0;
        dummyAssignments{i}(1:numel(negIndices)) = negIndices;
        constrainedCost{i}(1) =  sum(negValues);
    end

    % Step 2: Solve 2-D assignment problem
    % ------------------------------------
    multiplierSum = computeMultiplierSum({lagrangianMultipliers{1:end}},classToUse);
    twoDCost = constrainedCost{end};
    constrainedAssignment = getSolution(twoDCost,algFcn);
    dualCost = computeCostGivenAssignment(twoDCost,constrainedAssignment) + multiplierSum;
    violatedConstraint = true(numConstraints,1);
    bestDualCost = max(dualCost,bestDualCost);
    betaUpdateFlag = bestDualCost ~= dualCost;
            
    % Step 3: Add Constraints back
    % ----------------------------
    for i = numConstraints:-1:1
        currentDim = numConstraints - i + 1 + 2;
        
        if i == 1
            tempCostMatrix = costMatrix;
        else
            tempCostMatrix = constrainedCost{i-1};
        end
        
        % Compute gradient for violations
        gradients{i} = computeMultiplierGradient(costIndices{i},dimSize(currentDim),constrainedAssignment,classToUse);
        
        % If the dummy solution was picked, add assignments which led to
        % the dummy cost.
        validDummies = dummyAssignments{i}(dummyAssignments{i} > 1);
        gradients{i}(validDummies) = gradients{i}(validDummies) - 1;
        
        % Compute feasible costs
        multiplierSum = computeMultiplierSum({lagrangianMultipliers{i-1:-1:1}},classToUse);
        feasibleCostMatrix = constructFeasibleCostMatrix(tempCostMatrix,constrainedAssignment);
        % Get feasible solution.
        [enforcedConstraint,~,price{i}] = getSolution(feasibleCostMatrix,algFcn);
        
        % Enforce feasible solution on the assignment.
        indAvailable = enforcedConstraint(:,1);
        
        constrainedAssignment = constrainedAssignment(indAvailable,:);
        constrainedAssignment = [constrainedAssignment,enforcedConstraint(:,2)];  %#ok<AGROW>
        
        % Compute feasible cost
        feasibleCost(i) = computeCostGivenAssignment(tempCostMatrix,constrainedAssignment) + multiplierSum;
        
        bestFeasibleCost(i) = min(bestFeasibleCost(i),feasibleCost(i));

        if any(gradients{i})
            violatedConstraint(i) = false;
        end
    end
    updateSolution = bestFeasibleCost(1) == feasibleCost(1);

    % Step 4: Update Lagrangian Multipliers
    % -------------------------------------
    for i = numConstraints:-1:1
        % Update Lagrangian Multipliers
        if i == numConstraints
            thisDual = dualCost;
        else
            thisDual = feasibleCost(i+1);
        end
        if priceUpdate
            lagrangianMultipliers{i} = updateLagrangianMultiplierHeuristicPrice(lagrangianMultipliers{i},gradients{i},thisDual,feasibleCost(i),price{i});
        else
            currentDim = numConstraints - i + 1 + 2;
            % Hessians are p are set to eye and zero every nr iteration. Inform the
            % update function about it.
            % Subtract 1 from dimSize to get nr.
            iterFlag = mod(currentIter,dimSize(currentDim)-1) == 0;
            [lagrangianMultipliers{i},p{i},hessianMats{i},beta(i)] = updateLagrangianMultipler(lagrangianMultipliers{i},gradients{i},hessianMats{i},p{i},beta(i),thisDual,feasibleCost(i),iterFlag,betaUpdateFlag);
        end
    end
    
    % Step 5: Store best solution and update iteration information
    % ------------------------------------------------------------
    % Update iteration sweep
    currentIter = currentIter + 1;
    
    % Edge cases: 
    % 1. bestFeasibleCost = Inf
    % 2. bestDualCost = -Inf;
    % 3. bestDualCost ~= bestFeasibleCost. Then adding realmin to denominator can cause current gap to become negative.
    % 4. bestDualCost == bestFeasibleCost = +/-inf. currentGap will be NaN, which will be reduced to 0 by the max function
    currentGap = sign(bestFeasibleCost(1)) - bestDualCost/abs((bestFeasibleCost(1) + realmin(class(bestFeasibleCost))));
    currentGap = max(0,currentGap);
    bestGap = min(currentGap,bestGap);
    
    if updateSolution || currentIter == 2
        bestSolution = constrainedAssignment;
        bestGap = currentGap;
    end
    
    if all(violatedConstraint)
        bestGap = cast(0,classToUse);
        break;
    end
end % End of while loop

% Step 6: Return best solution found
% ----------------------------------
% First solution is a dummy. Accept it if the cost is negative.
if totalDummyCost >= 0
    assignments = bestSolution(2:end,:);
else
    assignments = bestSolution;
end
costMatrix(1) = totalDummyCost;
costOfAssignment = computeCostGivenAssignment(costMatrix,assignments);
solutionGap = bestGap;

end

%% Supporting Functions
% Computing Cost given assignments and multipliers
function cost = computeCostGivenAssignment(costMatrix,assignments,multiplier)
% This function takes a cost matrix of any dimension and assignments
% satisfying the dimensions of the costMatrix and returns the cost of total
% assignment.
s = size(costMatrix);
args = cell(numel(s),1);
for i = 1:numel(args)
    args{i} = assignments(:,i);
end
ind = sub2ind(s,args{:});
cost = sum(costMatrix(ind));
lSum = 0;
if nargin == 3
    lSum = computeMultiplierSum(multiplier,class(costMatrix));
end
cost = cost + lSum;
end

% Construct feasible cost matrix
function feasibleCostMatrix = constructFeasibleCostMatrix(costMatrix,assignments)
% This function generates the cost matrix to construct a feasible solution,
% given a constrained solution.
d1 = size(assignments,1);
s = size(costMatrix);
d2 = s(end);
feasibleCostMatrix = zeros(d1,d2,'like',costMatrix);
currentDim = numel(size(costMatrix)) - 1;

for i = 1:d1
    curArgs = cell(currentDim,1);
    for j = 1:currentDim
        curArgs{j} = assignments(i,j);
    end
    feasibleCostMatrix(i,:) = costMatrix(curArgs{1:currentDim},:);
end
end

% Compute gradient
function g = computeMultiplierGradient(sortedIndices,dimSize,assignments,classToUse)
% This function computes the gradient for each dimension. The gradient
% calculation is based on the following approach:
% 1. The index 1 has 0 gradient i.e. it is not punished for 0 or multiple
% assignments.
% 2. Other indices get gradient based on how many assignments contain them.
% For example, if index 5 is assigned 3 times, it's gradient is +2 i.e. it
% needs to be punished by adding cost. If it's assigned 1 times, 
% it's gradient is 0. If it's not assigned it's gradient is -1 i.e. it
% needs to be punished by subtracting cost.
indicesUsed = zeros(size(assignments,1)-1,1);
d1 = dimSize;
currentDim = numel(size(sortedIndices));
g = zeros(d1,1,classToUse);
args = cell(size(assignments,2),1);
for i = 1:numel(args)
    args{i} = assignments(2:end,i);
end
sIndices = sub2ind(size(sortedIndices),args{1:currentDim});
indicesUsed(:) = sortedIndices(sIndices);

for i = 2:numel(g)
    g(i) = (1 - numel(find(indicesUsed == i)));
end

end

% Lagrangian Multiplier Update via Accelerated Sub-gradient Descent
function [l,p,H,beta] = updateLagrangianMultipler(l,g,H,p,beta,dualCost,feasibleCost,iterFlag,betaFlag)
% The Lagrangian multipliers are updated using a accelerated sub-gradient
% method from [1].

classToUse = class(l(:));

% Method-specific constants
% Recommended values.
alpha = cast(2,classToUse);
ar = cast(0.02,classToUse);
br = cast(1.15,classToUse);

% Eq. (18)
if betaFlag
    beta = beta + 1;
else
    beta = max(beta - 1,1);
end
adaptCost = (1 + ar/beta^br)*dualCost;
thisGap = feasibleCost - adaptCost;
% When gap is infinite, take a small step. Detailed explanation in
% heuristic price update method.
thisGap(~isfinite(thisGap)) = 0.1*abs(adaptCost);

% Eq (17)
if ~iterFlag
    p = H*g;
    den = logical(g'*p);
    if den
        H = H + (1-1/alpha^2)*(p*p')/(g'*p);
    end
else
    H(:) = eye(size(H,1));
    p(:) = 0;
end

adaptFactor = (((alpha+1)/alpha)*thisGap*p)/(norm(g))^2;
if isfinite(adaptFactor)
    l(:) = l(:) + adaptFactor(:);
end

end

function l = updateLagrangianMultiplierHeuristicPrice(l,g,dualCost,feasibleCost,price)
% This function updates the Lagrangian multipliers using a Heuristic price
% update method described in [1].
thisGap = feasibleCost - dualCost;
% When feasibleCost is Inf, dualCost is some scalar, we need to decide how
% large a step to take in those cases. A huge step can greatly reduce the
% dualCost to a point beyond recovery. A small step is okay, but will take
% longer to converge, which is better than no convergence. Here, we assume
% that we can increase the dualCost by 1%.
thisGap(~isfinite(thisGap)) = 0.1*abs(dualCost);
gapFactor = (thisGap)/norm(g)^2;
dimSize = numel(price) - 1;
sumPrice = sum(price(:));
priceFactor = dimSize*price(:).*g(:);
adaptFactor = (gapFactor.*priceFactor)/sumPrice;
if isfinite(adaptFactor)
    l(:) = l(:) + adaptFactor(:);
end
end

% Get solution to assignment problem
function [assignments,cost,price] = getSolution(costMatrix,algFcn)
    [assignments,cost,price] = fusion.internal.assignment.assign2DRelaxed(costMatrix,algFcn);
end

function lSum = computeMultiplierSum(multiplier,classToUse)
% Compute the sum of lagrangian multipliers.
lSum = zeros(1,classToUse);
for i = 1:numel(multiplier)
    l = multiplier{i}(:);
    lSum = lSum + sum(l);
end
end

%% Input Parser
function [costMatrix,desiredGap,maxIterations,algFcn,priceUpdate] = parseAndValidateInputs(costMatrix,varargin)

funcName = mfilename;
validateattributes(costMatrix,{'double','single'},{'real','nonsparse','nonnan','nonempty'},funcName,'Cost Matrix',1);
classToUse = class(costMatrix);
numDims = numel(size(costMatrix));
coder.internal.errorIf(numDims<3, 'fusion:assignsd:expectedAtleast3D');

dimSize = size(costMatrix);
% Allow scalar values at the end only
if coder.target('MATLAB')
    coder.internal.errorIf(~(all(dimSize > 1)),'fusion:assignsd:expectedAtleast2Dims');
else
    cond1 = all(dimSize > 1);
    % Trailing ones are okay
    if ~cond1
        scalarDims = dimSize == 1;
        flipProd = logical(flip(cumprod(flip(scalarDims))));
        cond2 = all(flipProd == scalarDims);
    else
        cond2 = true;
    end
    coder.internal.errorIf(~cond2,'fusion:assignsd:expectedAtleast2Dims');
end

if nargin > 1
    validateattributes(varargin{1},{classToUse},{'scalar', 'finite', 'real', 'nonsparse','positive','<=',1},funcName,'Desired Gap',2);
    desiredGap = varargin{1};
else
    desiredGap = cast(0.01,classToUse);
end

if nargin > 2
    validateattributes(varargin{2}, {'numeric'}, ...
        {'nonsparse','real','finite','scalar','integer','positive'}, funcName, 'Max Iterations',3);
    maxIterations = varargin{2};
else
    maxIterations = 100;
end

% Assign algorithm for solving 2-D assignment problem.
validAlgs = {'munkres','auction','jv'};
validFcns = {@assignmunkres,@assignauction,@assignjv};
if nargin > 3 && ~isempty(varargin{3})
    algorithm = validatestring(varargin{3},validAlgs,funcName,'Algorithm',4);
else
    algorithm = 'auction';
end
algFcn = validFcns{strcmpi(algorithm, validAlgs)};

% Price update can happen only with assignauction algorithm.
if isequal(algFcn,validFcns{2})
    priceUpdate = true;
else
    priceUpdate = false;
end
end
