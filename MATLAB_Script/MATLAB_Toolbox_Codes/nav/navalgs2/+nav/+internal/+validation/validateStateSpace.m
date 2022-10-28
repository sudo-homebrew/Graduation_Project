function validateStateSpace(stateSpace, fcnName, varName)
%This function is for internal use only. It may be removed in the future.

%validateStateSpace Validate a state space object
%   All state space objects are derived from nav.StateSpace

%   Copyright 2019 The MathWorks, Inc.
    
    %#codegen
    validateattributes(stateSpace, {'nav.StateSpace'}, {'nonempty', 'scalar'}, ...
                       fcnName, varName); 

end