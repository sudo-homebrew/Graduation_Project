function validStateVec = validateStateVector(state, numStateVariables, fcnName, varName)
%This function is for internal use only. It may be removed in the future.

%validateStateVector Validate a single state vector
%   Restrict validations to simple shape checks to minimize
%   runtime overhead in performance-critical functions.

%   Copyright 2019 The MathWorks, Inc.

    %#codegen
    validateattributes(state, {'double'}, {'nonempty', 'vector', 'numel', numStateVariables}, ...
                       fcnName, varName);

    % Ensure it is always returned as a row vector
    validStateVec = state(:)';
end
