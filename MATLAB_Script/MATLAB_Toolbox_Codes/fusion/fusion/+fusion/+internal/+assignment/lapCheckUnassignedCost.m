function lapCheckUnassignedCost(costOfNonAssignment, costMatrix, assignFcnName)
% This function is for internal use only. It may be removed in the future.

% Check that the cost for no assignment is a scalar of the same type as
% the cost matrix.

% Copyright 2017 The MathWorks, Inc.

%#codegen

validateattributes(costOfNonAssignment, {class(costMatrix)}, ...
    {'scalar', 'finite', 'real', 'nonsparse'}, ...
    assignFcnName, 'COSTOFNONASSIGNMENT');
end
