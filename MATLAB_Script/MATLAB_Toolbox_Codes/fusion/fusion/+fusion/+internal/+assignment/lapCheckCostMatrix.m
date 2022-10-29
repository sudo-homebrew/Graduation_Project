function lapCheckCostMatrix(costMatrix,assignFcnName)
% This function is for internal use only. It may be removed in the future.

% Check that the cost matrix is a matrix of the correct type and size.

% Copyright 2017 The MathWorks, Inc.

%#codegen

validateattributes(costMatrix, {'single','double'}, ...
    {'real', 'nonsparse', 'nonnan', '2d'}, ...
    assignFcnName, 'COSTMATRIX');
end
