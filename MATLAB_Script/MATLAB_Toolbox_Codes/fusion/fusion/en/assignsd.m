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

