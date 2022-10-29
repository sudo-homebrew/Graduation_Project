function solutionList = kbestRankSolutions(solutionList)
% kbestRankSolutions - ranks the solutionList in an increasing order of 
% cost for k-best the algorithm to pick the top solution for partition.
% The solutionList{:}{3} element contains the cost of assignment of the
% solution.
% This is an internal function and may be removed in a future release.

% Copyright 2018 The MathWorks, Inc.

%#codegen

costs = zeros(numel(solutionList),1);
for i = 1:numel(solutionList)
    costs(i) = solutionList{i}{3}(1);
end
[~,I] = sort(costs);
if coder.target('MATLAB')
    solutionList = {solutionList{I}};
else
    tempSolutionList = solutionList;
    for i = 1:numel(I)
        solutionList{i} = tempSolutionList{I(i)};
    end
end
end