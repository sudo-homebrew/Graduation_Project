function preValidateNodeID(id, funcName, varName)
%This function is for internal use only. It may be removed in the future.

%preValidateNodeID Validate ONLY the basic attributes of nodeID input. 
%
%   preValidateNodeID(ID, FUNCNAME, VARNAME) checks whether the input, ID, is
%   a nonnegative, nonsparse, integer scalar. It however does not verify if
%   the nodeID exists in the factor definition or in the factor graph.
%
%   FUNCNAME and VARNAME are used in VALIDATEATTRIBUTES to construct the
%   error ID and message.

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    validateattributes(id, 'numeric', ...
        {'scalar', 'integer', 'nonnegative', 'nonsparse'}, funcName, varName);
end

