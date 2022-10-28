function clearedList = kbestRemoveDuplicates(solutionsList,costSize)
% kbestRemoveDuplicates: Remove duplicate solutions from the solutionList.
% solutionList - current list of solutions organized as:
% Format of solutionsList - a cell array
% 1st element: constrained cost matrix with padded rows and columns
% 2nd element: solution to constrained cost matrix
% 3rd element: cost of assignment
% 4th element: number of assignments in solutions which were enforced.
% 
% costSize - size of the original cost matrix.
% 
% This is an internal function and may be removed in a future release.

% Copyright 2018 The MathWorks, Inc.

%#codegen

% Find solutions that are duplicates of each other and keep only one copy.
toClearIndex = false(numel(solutionsList),1);

% Start for duplicate search from top to bottom. This will ensure the
% duplicate with higher constraints get deleted. 
for i = 1:numel(solutionsList)
    assignment1 = solutionsList{i}{2};
    % The solution can contain unassigned rows and columns
    % as padded assignments. Remove them before comparison.
    if numel(costSize) == 2
        assignment1 = sortrows(assignment1);
        assignment1 = fusion.internal.assignment.kbestRemoveUnassigned(assignment1,costSize);
    end
    for j = i+1:numel(solutionsList)
        assignment2 = solutionsList{j}{2};
        % The solution can contain unassigned rows and columns
        % as padded assignments. Remove them before comparison.
        if numel(costSize) == 2
            assignment2 = sortrows(assignment2);
            assignment2 = fusion.internal.assignment.kbestRemoveUnassigned(assignment2,costSize);
        end
        if isequal(assignment1, assignment2)
            if solutionsList{i}{4} <= solutionsList{j}{4}
                toClearIndex(j) = true;
            else
                toClearIndex(i) = true;
            end
        end
    end
end
if coder.target('MATLAB')
    clearedList = solutionsList(~toClearIndex);
else
    clearedList = repmat(solutionsList,[1 0]);
    coder.varsize('clearedList',[1 inf],[0 1]);
    for i = 1:numel(toClearIndex)
        if ~toClearIndex(i)
            clearedList{end+1} = solutionsList{i};
        end
    end
end
end