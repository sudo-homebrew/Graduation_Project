classdef JVProblemSolutionPair < fusion.internal.assignment.AbstractProblemSolutionPair
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % This class implements the ProblemSolutionPair interface for the JV
    % algorithm. It provides a lower bound on the best solution and sorts
    % the queue with lower bound. This is based on the heuristics provided
    % by Cox & Miller [1].
    
    % References:
    %
    % [1] Miller, Matt L., Harold S. Stone, and Ingemar J. Cox. "Optimizing
    % Murty's ranked assignment method." IEEE Transactions on Aerospace and
    % Electronic Systems 33.3 (1997): 851-862.
    
    %#codegen
    properties
        ColReduction
        RowReduction
        LowerBound
    end
    
    methods
        function obj = JVProblemSolutionPair(costMatrix, costOfNonAssignment)
            obj@fusion.internal.assignment.AbstractProblemSolutionPair(costMatrix, costOfNonAssignment);
            problemSize = size(costMatrix,1) + size(costMatrix,2);
            obj.ColReduction = zeros(1,problemSize,'like',costMatrix);
            obj.RowReduction = zeros(problemSize,1,'like',costMatrix);
            obj.LowerBound = inf(1,'like',costMatrix);
        end
        
        function minSlack = computeMinimumSlack(obj, tuple)
            l = obj.PaddedCostMatrix(tuple(1),:);
            l(tuple(2)) = inf;
            ui = obj.RowReduction(tuple(1));
            vh = obj.ColReduction;
            minSlack = min(l - ui - vh);
        end
        
        function slacks = computeSlacks(obj, tuples)
            slacks = zeros(size(tuples,1),1,'like',obj.PaddedCostMatrix);
            for i = 1:size(tuples,1)
                slacks(i) = computeMinimumSlack(obj, tuples(i,:));
            end
        end
        
        function obj = solveProblem(obj)
            % Compute linear assignment
            [obj.RowSoln, obj.ColSoln, obj.ColReduction] = fusion.internal.assignment.lapDijkstra(obj.PaddedCostMatrix,obj.RowSoln,obj.ColSoln,obj.ColReduction);
            obj.RowReduction = min(bsxfun(@minus,obj.PaddedCostMatrix, obj.ColReduction),[],2);
        end
        
        function objArray = partition(obj)
            % Start with the input Problem solution pair
            P = obj;
            % Valid assignments for partitioning
            isValid = ~obj.IsEnforced & ~obj.IsDummySolution;
            
            % Pre-define local variables for integer arithmetic in
            % generated code.
            n = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntLogicalSum(isValid);
            nMax = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntSize2D(obj.Assignment);
            assert(n <= nMax);  % Assist upper bound analysis
            idx = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntZero();
            ONE = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();
            S = coder.nullcopy(matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntIndex(zeros(n,2)));
                        
            S(:) = obj.Assignment(isValid,:);
            objArray = cell(n,1);
            for i = ONE:n
                slacks = computeSlacks(P, S);
                [maxSlack,idx(1)] = max(slacks);
                highestTuple = S(idx,:);
                Phat = P;
                Phat = removeTuple(Phat,highestTuple);
                Phat.LowerBound = P.BestSolutionCost + maxSlack;
                Phat.IsSolved = false;
                objArray{i} = Phat;
                P = enforceTuple(P,highestTuple);
                S(idx,:) = [];
            end
        end

        function cost = getCostToSort(obj)
            cost = max(obj.BestSolutionCost,obj.LowerBound);
        end
    end
end

