classdef AuctionProblemSolutionPair < fusion.internal.assignment.AbstractProblemSolutionPair
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    properties
        BigNumber
    end
    
    methods
        function obj = AuctionProblemSolutionPair(costMatrix, costOfNonAssignment)
            obj@fusion.internal.assignment.AbstractProblemSolutionPair(costMatrix, costOfNonAssignment);
            maxVal = max(costMatrix(isfinite(costMatrix)),[],'all');
            obj.BigNumber = 1e10*maxVal;
        end
    end
    
    methods
        function obj = solveProblem(obj)
            % Compute linear assignment. In case of "auction", the internal
            % forward-reverse auction is not used, rather a re-padding with
            % a large cost of unassignment is used. This is because the
            % forward-reverse auction is suspectible to getting stuck in
            % auction-phases when the scaling of cost matrix is poor.
            bigNumber = obj.BigNumber;
            assignments = assignauction(obj.PaddedCostMatrix,bigNumber);
            n = sum(obj.CostSize);
            classToUse = class(obj.PaddedCostMatrix);
            [obj.RowSoln, obj.ColSoln] = obj.sortedAssignmentToRowColSoln(assignments,n,classToUse);
        end
    end
end