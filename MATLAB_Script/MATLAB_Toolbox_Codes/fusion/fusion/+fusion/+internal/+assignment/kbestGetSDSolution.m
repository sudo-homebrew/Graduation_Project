function [soln, cost, gap, isValid] = kbestGetSDSolution(costMatrix, desiredGap, maxIterations, algorithm)
% kbestGetSDSolution - Calculates solution for S-D assignment problem 
% 
% Inputs:
% costMatrix is the S-D cost matrix 
% desiredGap is the gap that controls the assignsd convergence
% maxItertions is the maximum number of iterations that assignsd runs
% algorithm is the 2-D solution algorithm used by assignsd
%
% Outputs:
% soln - A P-by-S list of assignments (S - dimension of costMatrix).
% cost - cost of assignment of solution.
% gap - duality gap of the solution.
% isValid - A flag indicating if the solution is valid.
%
% This is an internal function and may be removed in a future release.

% Copyright 2018 The MathWorks, Inc.

%#codegen

% First, get the S-D solution
[soln, cost, gap] = assignsd(costMatrix, desiredGap, maxIterations, algorithm);

% Valid solutions must have finite cost.
isValid = isfinite(cost);
