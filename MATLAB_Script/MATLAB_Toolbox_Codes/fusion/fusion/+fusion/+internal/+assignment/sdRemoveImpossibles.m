function costMatrix = sdRemoveImpossibles(costMatrix)
% This is an internal function and may be removed in a future release.
%
% costMatrix = sdRemoveImpossibles(costMatrix) removes impossibles from a
% costMatrix. It gates the impossible assignments by setting their values 
% to Inf.
% The impossibles are defined like this:
% For a 5-D matrix:
% Feasible assignment: cost(4,5,6,7,8) - cost(4,1,1,1,1) - cost(1,5,1,1,1) - cost(1,1,6,1,1) - cost(1,1,1,7,1) - cost(1,1,1,1,8) must be less than 0.

% Copyright 2018 The MathWorks, Inc.

%#codegen
temporaryMatrix = costMatrix;

dimSize = size(costMatrix);
numDims = numel(dimSize);

if coder.target('MATLAB')
    oneArgs = num2cell(ones(numDims,1));
else
    oneArgs = cell(numDims,1);
    for i = 1:numDims
        oneArgs{i} = 1;
    end
end
coder.unroll();
for i = 1:numDims
    thisArg = oneArgs;
    thisArg{i} = 1:dimSize(i);
    temporaryMatrix = bsxfun(@minus,temporaryMatrix,temporaryMatrix(thisArg{:}));
end
indices = temporaryMatrix > 0;
costMatrix(indices) = Inf;
