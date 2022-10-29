classdef MunkresProblemSolutionPair < fusion.internal.assignment.AbstractProblemSolutionPair
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    methods
        function obj = solveProblem(obj)
            assignments = fusion.internal.assignment.hungarianAssignment(obj.PaddedCostMatrix);
            [i, j] = find(assignments);
            [~,idx] = sort(i);
            obj.RowSoln = j(idx);
            obj.ColSoln = i;
        end
    end
end