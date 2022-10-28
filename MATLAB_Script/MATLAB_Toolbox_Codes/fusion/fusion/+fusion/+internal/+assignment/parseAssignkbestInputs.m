function [costMatrix, costOfNonAssignment, numSolutions, algorithm] = parseAssignkbestInputs(varargin)
% This is an internal function and may be removed or modified in a future
% release. 

% Copyright 2021 The MathWorks, Inc.

%#codegen

% Main function name
fName = 'assignkbest';

% Parses and validates the inputs to the assignkbest function
narginchk(2,4);

% validate costMatrix
fusion.internal.assignment.lapCheckCostMatrix(varargin{1},fName);
costMatrix = varargin{1};

[nRow, nCol] = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntSize2D(costMatrix);
numSolutions = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntZero();

% validate cost of non-assignment
validateattributes(varargin{2},{'numeric','cell'},{},fName,'costOfNonAssignment');
classToUse = class(costMatrix);
if iscell(varargin{2})
    validateattributes(varargin{2},{'cell'},{'numel',2},fName,'costOfNonAssignment');
    cRow = varargin{2}{1};
    validateattributes(cRow,{classToUse},{'nonsparse','real','finite','numel',nRow},fName,'costOfRowNonAssignment');
    cCol = varargin{2}{2};
    validateattributes(cCol,{classToUse},{'nonsparse','real','finite','numel',nCol},fName,'costOfColNonAssignment');
else
    if isscalar(varargin{2})
        validateattributes(varargin{2},{classToUse},{'nonsparse','real','finite'},fName,'costOfNonAssignment');
    else
        validateattributes(varargin{2},{classToUse},{'nonsparse','real','finite','numel',2},fName,'costOfNonAssignment');
    end
end

costOfNonAssignment = varargin{2};

% Validate k
if nargin > 2
    validateattributes(varargin{3}, {'numeric'}, ...
        {'nonsparse','real','finite','scalar','integer'}, fName, 'k');
    numSolutions(1) = varargin{3};
else
    numSolutions(1) = 1;
end

validAlgs = {'munkres', 'jv', 'auction', 'matchpairs'};
if nargin>3
    % Find the function to use in the list of all valid algorithms
    algorithm = validatestring(varargin{4}, validAlgs, fName, 'Algorithm');
else
    % Default: "jv"
    algorithm = 'jv';
end

end
