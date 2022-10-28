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

