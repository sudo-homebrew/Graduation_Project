function validateStateMatrix(state, numRows, numStateVariables, fcnName, varName)
%This function is for internal use only. It may be removed in the future.

%validateStateMatrix Validate a matrix of multiple states
%   Restrict validations to simple shape checks to minimize
%   runtime overhead in performance-critical functions.

%   Copyright 2019 The MathWorks, Inc.

    %#codegen
    validateattributes(state, {'double'}, {'nonempty', 'size', [numRows numStateVariables]}, ...
                       fcnName, varName);

end
