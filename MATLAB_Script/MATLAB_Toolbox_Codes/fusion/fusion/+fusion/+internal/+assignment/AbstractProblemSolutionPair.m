classdef (Abstract) AbstractProblemSolutionPair
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % Copyright 2020 The MathWorks, Inc.
    
    %#codegen
    
    properties
        % PaddedCostMatrix Square cost matrix after padding cost of non
        % assignment
        PaddedCostMatrix
        
        % RowSoln Solution of the assignment problem from row perspective
        RowSoln
        
        % ColnSoln Solution of the assignment problem from column
        % perspective
        ColSoln
        
        % A vector of flags controlling if the assignment in RowSoln is
        % enforced.
        IsEnforced
        
        % A 2x1 vector storing the actual size of the cost matrix without
        % padding
        CostSize
        
        % A scalar to represent the cost of the optimal solution to the
        % problem
        BestSolutionCost
        
        % A vector of flags controlling if the assignment in RowSoln is a
        % dummy-to-dummy solution and should not be partitioned upon.
        IsDummySolution
    end
    
    properties (Access = protected)
        IsSolved
    end
    
    properties (SetAccess = protected, Dependent)
        Assignment
    end
    
    methods
        function val = get.Assignment(obj)
            % Get sorted assignment value using RowSoln property
            n = obj.CostSize(1) + obj.CostSize(2);
            rowIdx = cast((1:n)','like',obj.RowSoln);
            val = [rowIdx obj.RowSoln];
        end
    end
    
    % Abstract method for algorithm-specific problem/solution pair
    methods (Abstract)
        solvedObj = solveProblem(obj);
    end
    
    methods
        function obj = AbstractProblemSolutionPair(costMatrix, costOfNonAssignment)
            % Pad the cost matrix with cost of unassignment
            paddedMatrix = fusion.internal.assignment.lapPadForUnassignedRowsAndColumns(costMatrix,costOfNonAssignment);
            [nRow, nCol] = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntSize2D(costMatrix);         
            obj.CostSize = [nRow nCol];
            problemSize = nRow + nCol;
            
            % Initialize the object
            obj.PaddedCostMatrix = paddedMatrix;
            obj.RowSoln = nan(problemSize, 1,'like',costMatrix);
            obj.ColSoln = nan(1,problemSize,'like',costMatrix);
            obj.IsEnforced = false(problemSize, 1);
            obj.IsDummySolution = false(problemSize, 1);
            obj.IsSolved = false;
            obj.BestSolutionCost = inf(1,'like',costMatrix);
        end
        
        function obj = enforceTuple(obj, tuple)
            % Take a ProblemSolutionPair and enforce that its solution must
            % always contain tuple. tuple is a 2-by-1 vector defining an
            % assignment tuple(1) -> tuple(2);
            temp = obj.PaddedCostMatrix(tuple(1),tuple(2));
            obj.PaddedCostMatrix(:,tuple(2)) = inf;
            obj.PaddedCostMatrix(tuple(1),:) = inf;
            obj.PaddedCostMatrix(tuple(1),tuple(2)) = temp;
            obj.RowSoln(tuple(1)) = tuple(2);
            obj.ColSoln(tuple(2)) = tuple(1);
            obj.IsEnforced(tuple(1)) = true;
        end
        
        function obj = removeTuple(obj, tuple)
            % Take a ProblemSolutionPair and enforce that its solution must
            % NOT contain tuple. tuple is a 2-by-1 vector defining an
            % assignment tuple(1) -> tuple(2);
            obj.PaddedCostMatrix(tuple(1),tuple(2)) = inf;
            obj.RowSoln(tuple(1)) = nan;
            obj.ColSoln(tuple(2)) = nan;
        end
        
        function [assignments, unassignedRows, unassignedCols, cost] = formatSolution(obj)
            % Format the solution as assignments similar to assignment
            % algorithms
            [assignments, unassignedRows, unassignedCols] = fusion.internal.assignment.kbestRemoveUnassigned(uint32(obj.Assignment),obj.CostSize);
            assignments = sortrows(assignments);
            cost = obj.BestSolutionCost;
        end
        
        function tf = isValid(obj)
            % A flag to determine if the problem/solution pair is valid
            % i.e. must be partitioned or returned.
            tf = all(isfinite(obj.RowSoln)) && all(isfinite(obj.ColSoln(:))) && isfinite(obj.BestSolutionCost(1));
        end
        
        function obj = solve(obj)
            % solvedObj = solve(obj) will solve the problem and return the
            % solved problem/solution pair.
            
            % Only invest in solving if the problem is feasible doing a
            % quick sanity check.
            if isFeasible(obj)
                obj = solveProblem(obj);
            end
            
            % If returned solution is finite
            if all(isfinite(obj.RowSoln))
                % Compute cost of the solution
                assignment = obj.Assignment;
                % Cast sz to type of RowSoln (float/double) before comparison
                sz = cast(obj.CostSize,'like',obj.RowSoln);
                obj.IsDummySolution = obj.RowSoln > sz(2) & assignment(:,1) > sz(1);
                ind = sub2ind(size(obj.PaddedCostMatrix),assignment(:,1),assignment(:,2));
                cost = obj.PaddedCostMatrix(:);
                obj.BestSolutionCost = sum(cost(ind(:)));
            else
                obj.BestSolutionCost = cast(inf,'like',obj.BestSolutionCost);
            end
            obj.IsSolved = true;
        end

        function objArray = partition(obj)
            % Start with the input Problem solution pair
            P = obj;
            
            % Only partition on assignments which were not enforced or
            % which were not dummy-to-dummy
            S = obj.Assignment(~obj.IsEnforced & ~obj.IsDummySolution,:);
            
            % Go through each tuple and initiate a new p/s pair
            n = size(S,1);
            objArray = cell(n,1);
            for i = 1:n
                tupleToRemove = S(i,:);
                Phat = P;
                Phat = removeTuple(Phat,tupleToRemove);
                Phat = solve(Phat);
                objArray{i} = Phat;
                P = enforceTuple(P,tupleToRemove);
            end
        end
        
        function tf = isFeasible(obj)
            % tf = isFeasible(obj) returns if the p/s pair upon solving
            % will return a feasible solution.
            isFiniteMatrix = isfinite(obj.PaddedCostMatrix);
            tf = all(sum(isFiniteMatrix,1) > 0) && all(sum(isFiniteMatrix,2) > 0);
        end
        
        function tf = isSolved(obj)
            % tf = isSolved(obj) returns if a p/s pair is solved.
            tf = obj.IsSolved;
        end

        function cost = getCostToSort(obj)
            cost = obj.BestSolutionCost;
        end
    end
    
    methods (Access = protected, Static)       
        function [rowSoln, colSoln] = sortedAssignmentToRowColSoln(assignments,n,classToUse)
            rowSoln = zeros(n,1,classToUse);
            colSoln = zeros(1,n,classToUse);
            i = assignments(:,1);
            j = assignments(:,2);
            nFound = numel(i);
            rowSoln(1:nFound) = j;
            rowSoln((nFound+1):end) = nan;
            [~,idx] = sort(j);
            colSoln(1:nFound) = i(idx);
            colSoln((nFound+1):end) = nan;
        end
    end
end