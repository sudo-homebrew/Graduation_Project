classdef ProblemSolutionQueue
    % This is an internal class and may be removed or modified in a future
    % release.
    
    % Copyright 2020 The MathWorks, Inc.
    
    % A manager for managing the partition solution pairs
    
    %#codegen
    properties
        NumProblems
    end
    
    properties (Access = {?matlab.unittest.TestCase,...
            ?fusion.internal.assignment.ProblemSolutionQueue})
        MaxNumSubProblems;
        AllProblemList
        pClassToUse
    end
    
    methods
        function obj = ProblemSolutionQueue(problem, maxNumSubProblems)
            coder.internal.prefer_const(maxNumSubProblems);
            obj.MaxNumSubProblems = maxNumSubProblems;
            sampleProblem = problem;
            problemList = repmat({sampleProblem},obj.MaxNumSubProblems,1);
            obj.AllProblemList = problemList;
            obj.NumProblems = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntZero();
            obj.pClassToUse = class(problem.PaddedCostMatrix);
        end
        
        function [obj, topProblem, isValidProblem] = extractBestSolution(obj)
            % It is assumed that the queue is ordered when this method is
            % scalled.

            % Current problem of top
            topProblem = obj.AllProblemList{1};
            
            % We need the top problem to be solved.
            % As soon as the top problem is solved, it may go down in the
            % list of problems, if it is not ordered by its best solution
            % cost yet.
            while ~isSolved(topProblem)
                % Solve the top problem
                topProblem = solve(topProblem);
                
                % If top problem is valid, keep it and resort the list
                if isValid(topProblem)
                    obj.AllProblemList{1} = topProblem;
                    obj = sortQueue(obj);
                else
                    obj = removeTopProblem(obj);
                end
                
                % The queue may be empty now after removing the problem and
                % no valid top problem has been found yet.
                if isDone(obj)
                    isValidProblem = false;
                    return;
                end
                % Get the current top problem and restart the while loop
                topProblem = obj.AllProblemList{1};
            end
            
            % Remove the top problem from the queue. The queue is already
            % sorted.
            if isValid(topProblem)
                obj = removeTopProblem(obj);
                isValidProblem = true;
            else
                isValidProblem = false;
            end
        end

        function obj = removeTopProblem(obj)
            for i = 1:obj.NumProblems-1
                obj.AllProblemList{i} = obj.AllProblemList{i+1};
            end
            obj.NumProblems = obj.NumProblems - 1;
        end
        
        function obj = setProblems(obj, problemList)
            if isMATLAB()
                obj.AllProblemList(1:obj.NumProblems) = problemList;
            else
                for i = 1:obj.NumProblems
                    obj.AllProblemList{i} = problemList{i};
                end
            end
        end
        
        function problemList = getProblems(obj)
            if isMATLAB()
                problemList = obj.AllProblemList(1:obj.NumProblems);
            else
                n = obj.NumProblems;
                assert(n <= obj.MaxNumSubProblems);
                problemList = repmat({obj.AllProblemList{1}},n,1);
                for i = 1:numel(problemList)
                    problemList{i} = obj.AllProblemList{i};
                end
            end
        end
        
        function obj = addProblems (obj, problems)
            % addProblems(obj, problems) adds a list of problems to the
            % queue. problems is a cell array of ProblemSolutionPair.
            n = obj.NumProblems;
            if isMATLAB()
                nIn = numel(problems);
                obj.AllProblemList(n + (1:nIn)) = problems;
            else
                ONE = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntOne();
                nIn = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntNumel(problems);
                for i = ONE:nIn
                    obj.AllProblemList{n + i} = problems{i};
                end
            end
            obj.NumProblems = obj.NumProblems + nIn;
        end
        
        function tf = isDone(obj)
            % A method to determine if queue is empty
            tf = obj.NumProblems == 0;
        end
        
        function obj = sortQueue(obj)
            % sortQueue(obj) sorts the queue in order. The order is
            % determined by ProblemSolutionPair. For all assignments except
            % JV, the order is determined by the cost of the best solution
            % to the problem. For JV, the order is determined by the lower
            % bound of the best solution.
            if coder.target('MATLAB')
                costsToSort = cellfun(@(x)getCostToSort(x),obj.AllProblemList(1:obj.NumProblems));
            else
                n = obj.NumProblems;
                assert(n <= obj.MaxNumSubProblems);
                costsToSort = zeros(n,1,obj.pClassToUse);
                for i = 1:obj.NumProblems
                    costsToSort(i) = getCostToSort(obj.AllProblemList{i});
                end
            end
            [~,idx] = sort(costsToSort);
            if coder.target('MATLAB')
                problemList = obj.AllProblemList(1:obj.NumProblems);
                obj.AllProblemList(1:obj.NumProblems) = problemList(idx);
            else
                obj.AllProblemList = cellArrayIdxMemoryEfficient(obj.AllProblemList,idx);
            end
        end
        
        function [assignments, unassignedRows, unassignedCols, cost] = formatSolution(obj)
            % Format the solution as assignments, unassigned rows and
            % unassigned columns.
            n = obj.NumProblems;
            assert(n <= obj.MaxNumSubProblems);
            assignments = cell(n,1);
            unassignedRows = cell(n,1);
            unassignedCols = cell(n,1);
            cost = zeros(n,1,obj.pClassToUse);
            for i = 1:n
                problem = obj.AllProblemList{i};
                [assignments{i}, unassignedRows{i}, unassignedCols{i}, cost(i)] = formatSolution(problem);
            end
        end
    end
    
    methods (Static, Hidden)
        function props = matlabCodegenSoftNontunableProperties(~)
            props = {'MaxNumSubProblems','pClassToUse'};
        end
    end
end

function objArray = cellArrayIdxMemoryEfficient(objArray, sortedIdx)
for i = 1:numel(sortedIdx)
    tmp = objArray{i};
    objArray{i} = objArray{sortedIdx(i)};
    objArray{sortedIdx(i)} = tmp;
    sortedIdx(sortedIdx == i) = sortedIdx(i);
end
end

function tf = isMATLAB()
tf = coder.target('MATLAB');
end