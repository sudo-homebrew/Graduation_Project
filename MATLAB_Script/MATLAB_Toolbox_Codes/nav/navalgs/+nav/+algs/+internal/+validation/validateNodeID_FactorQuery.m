function validateNodeID_FactorQuery(id, idCandidates, funcName, varName, varargin)
%This function is for internal use only. It may be removed in the future.

%validateNodeID_FactorQuery Validate node ID input. This validation function 
%   is only to be used in a member method of a factor object.
%
%   validateNodeID(ID, IDCANDIDATES, FUNCNAME, VARNAME) checks whether
%   the input, ID, is a valid node ID number at factor query time. Here a 
%   valid node ID is expected to be a nonnegative integer scalar that 
%   matches one of the entries in IDCANDIDATES, which is defined at the
%   factor construction time.
%
%   FUNCNAME and VARNAME are used in VALIDATEATTRIBUTES to construct the
%   error ID and message.
%
%   validateNodeID_FactorQuery(___, VARARGIN) allows the user to specify 
%   additional attributes supported in VALIDATEATTRIBUTES in a cell array, 
%   VARARGIN.

%   Copyright 2021 The MathWorks, Inc.

%#codegen

    validateattributes(id, 'numeric', ...
        {'scalar', 'integer', 'nonnegative', 'nonsparse', varargin{:}}, funcName, varName);
    coder.internal.errorIf(isempty(find(id == idCandidates, 1)),...
        'nav:navalgs:factors:NodeIDNotFoundInFactor');
end

