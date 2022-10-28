function x = switchimm(modelType1, x1, modelType2,x2)
%SWITCHIMM Model conversion function for trackingIMM object
%   x = SWITCHIMM(modelType1, x1, modelType2) converts the State or
%   StateCovariance from modelType1 state definition to modelType2 state definition.
%   modelType1 - specifies the string name of the current motion model.
%   x1         - specifies State or StateCovariance corresponding to modelType1
%   modelType2 - specifies the string name of motion model to which x1 needs to be converted.
%
%   x = SWITCHIMM(...,x2), additionally, lets you specify the size and type of the
%   output. When not specified, x has the same type and dimensionality as x1.
%   x2 specifies State or StateCovariance corresponding to modelType2.
%
%   x1 - a L-by-1 real vector or L-by-L real matrix in modelType1 state space.
%   x  - a M-by-1 real vector or M-by-M real matrix in modelType2 state space.
%
%   Notes:
%   ------
%   1. x2 is an optional input parameter. By default, the function preserve
%   the dimensionality of x1 to convert the state or stateCovariance.
%   2. x2 is a M-by-1 real vector or M-by-M real matrix in modelType2 state
%   space.
%
%   % Example 1: Converting state from constant acceleration to constant
%   % turn with x2 as an input parameter
%   % --------------------------------------------------------------------
%   %   The input parameters for switchimm are
%   modelType1 = 'constacc';
%   modelType2 = 'constturn';
%   x1 = [1;2;3;4;5;6];
%   x2 = [0;0;0;0;0;0;0];
%   x  = switchimm(modelType1,x1,modelType2,x2)
%
%   %   In this case, the model conversion function converts the 2-D
%   %   constant acceleration input to a 3-D constant-turn model output.
%   %   Size and class of x will always be same as that of x2 when x2 is
%   %   given in input parameter.
%
%   % Example 2: Converting state from constant acceleration to constant
%   % velocity without x2 as an input parameter
%   % --------------------------------------------------------------------
%   %   The input parameters for switchimm are
%   modelType1 = 'constacc';
%   modelType2 = 'constvel';
%   x1 = single([1;2;3;4;5;6]);
%   x = switchimm(modelType1,x1,modelType2)
%
%   %   In this case, the output is a 2-D constant velocity state, because
%   %   the input is a 2-D constant acceleration. Class and Dimensionality
%   %   of x will always be same as x1 when x2 is not the input parameter.
%
%   See also: trackingIMM, initekfimm, constvel, constacc, constturn.

%   Copyright 2018 The MathWorks, Inc.

%#codegen
% Validate inputs
narginchk(3,4);
if (nargin < 4)

    [classToUse,x3,x4] = inputValidation(modelType1, x1, modelType2);
else

    [classToUse,x3,x4] = inputValidation(modelType1, x1, modelType2, x2);
end

switch lower(modelType1)
    case 'constvel'
        switch lower(modelType2)
            case 'constvel'
                x = fusion.internal.constvel2constvel(x3,x4);
            case 'constacc'
                x = fusion.internal.constvel2constacc(x3,x4);
            case 'constturn'
                x = fusion.internal.constvel2constturn(x3,x4);
            otherwise
                coder.internal.error('fusion:trackingIMM:invalidModel',modelType2);
        end
    case 'constacc'
        switch lower(modelType2)
            case 'constvel'
                x = fusion.internal.constacc2constvel(x3,x4);
            case 'constacc'
                x = fusion.internal.constacc2constacc(x3,x4);
            case 'constturn'
                x = fusion.internal.constacc2constturn(x3,x4);
            otherwise
                coder.internal.error('fusion:trackingIMM:invalidModel',modelType2);
        end
    case 'constturn'
        switch lower(modelType2)
            case 'constvel'
                x = fusion.internal.constturn2constvel(x3,x4);
            case 'constacc'
                x = fusion.internal.constturn2constacc(x3,x4);
            case 'constturn'
                x = fusion.internal.constturn2constturn(x3,x4);
            otherwise
                coder.internal.error('fusion:trackingIMM:invalidModel',modelType2);
        end
    otherwise
        coder.internal.error('fusion:trackingIMM:invalidModel',modelType1);
end
x = cast(x,classToUse);
end

%-----------------------------------------------------------------------
% Helper function
%------------------------------------------------------------------------
function [classToUse,x3,x4] = inputValidation(modelType1, x1, modelType2, x2)
if coder.target('MATLAB')
    modelType1 = localValidateString(lower(modelType1));
    modelType2 = localValidateString(lower(modelType2));
else
    modelType1 = validatestring(modelType1, {'constvel','constacc','constturn'}, 'switchimm', 'modelType1', 1);
    modelType2 = validatestring(modelType2, {'constvel','constacc','constturn'}, 'switchimm', 'modelType2', 3);
end

validateattributes(x1, {'single','double'},{'real', 'finite', 'nonsparse'},...
    'switchimm', 'x1', 2);
if isvector(x1)
    x3 = x1(:);
else
    x3 = x1;
end
if ~isvector(x3)
    %Validating the State Covariance
    tol = 100 * eps(max(abs(diag(x3))));
    coder.internal.errorIf(any((abs(x3-x3')>sqrt(tol)),'all'), 'shared_tracking:KalmanFilter:invalidCovarianceValues', 'x2');
end
switch lower(modelType1)
    case 'constvel'
        cond = (size(x3,1)~=2 && size(x3,1)~=4 && size(x3,1)~=6);
        if ~isvector(x3)
            coder.internal.errorIf(cond, 'fusion:trackingIMM:incorrectStateCovarianceSize');
        else
            coder.internal.errorIf(cond, 'fusion:trackingIMM:incorrectStateSize');
        end
    case 'constacc'
        cond =(size(x3,1)~=3 && size(x3,1)~=6 && size(x3,1)~=9);
        if ~isvector(x3)
            coder.internal.errorIf(cond, 'fusion:trackingIMM:incorrectStateCovarianceSize');
        else
            coder.internal.errorIf(cond, 'fusion:trackingIMM:incorrectStateSize');
        end

    case 'constturn'
        cond = (size(x3,1)~=5 && size(x3,1)~=7);
        if ~isvector(x3)
            coder.internal.errorIf(cond, 'fusion:trackingIMM:incorrectStateCovarianceSize');
        else
            coder.internal.errorIf(cond, 'fusion:trackingIMM:incorrectStateSize');
        end
end
if(nargin<4)
    [newClass,newRef] = referenceState(modelType1, x3, modelType2);
    x2         = newRef;
    classToUse = newClass;
else
    classToUse = class(x2);
end

validateattributes(x2, {'single','double'},{'real', 'finite', 'nonsparse'},...
    'switchimm', 'x2', 4);
if(~isvector(x1) && ~isvector(x2))
    x4 = x2;
else
    x4 = x2(:);
end
if ~isvector(x4)
    %Validating the State Covariance
    tol = 100 * eps(max(abs(diag(x4))));
    coder.internal.errorIf(any((abs(x4-x4')>sqrt(tol)),'all'), 'shared_tracking:KalmanFilter:invalidCovarianceValues', 'x2');
end
switch lower(modelType2)
    case 'constvel'
        cond =(size(x4,1)~=2 && size(x4,1)~=4 && size(x4,1)~=6);
        % Validating the size of State and State Covariance
        if ~isvector(x4)
            coder.internal.errorIf(cond, 'fusion:trackingIMM:incorrectStateCovarianceSize');
        else
            coder.internal.errorIf(cond, 'fusion:trackingIMM:incorrectStateSize');
        end
    case 'constacc'
        cond = (size(x4,1)~=3 && size(x4,1)~=6 && size(x4,1)~=9);
        % Validating the size of State and State Covariance
        if ~isvector(x4)
            coder.internal.errorIf(cond, 'fusion:trackingIMM:incorrectStateCovarianceSize');
        else
            coder.internal.errorIf(cond, 'fusion:trackingIMM:incorrectStateSize');
        end
    case 'constturn'
        cond = (size(x4,1)~=5 && size(x4,1)~=7);
        if ~isvector(x4)
            coder.internal.errorIf(cond, 'fusion:trackingIMM:incorrectStateCovarianceSize');
        else
            coder.internal.errorIf(cond, 'fusion:trackingIMM:incorrectStateSize');
        end
end
end


function x2 = dims2State(dims1,model2,doCov,classToUse)
% Calculating x2 if it is not given in input
if(isequal(model2 , 'constvel'))
    numStates = dims1*2;
elseif(isequal(model2,'constacc'))
    numStates = dims1*3;
else
    if isequal(dims1,3)
        numStates = 7;
    else
        numStates = 5;
    end
end
if doCov
    x2 = zeros(numStates,numStates,classToUse);
else
    x2 = zeros(numStates,1,classToUse);
end
end

function [classToUse,newRef] = referenceState(model1, x1, model2)
classToUse  = class(x1);
doCov       = ~isvector(x1);
%Finding the dimension of x1.
if(isequal(model1 , 'constvel'))
    dim = fusion.internal.constveldims(x1);
elseif(isequal(model1,'constacc'))
    dim = fusion.internal.constaccdims(x1);
elseif(isequal(model1,'constturn'))
    dim = fusion.internal.constturndims(x1);
else
    coder.internal.error('fusion:trackingIMM:invalidConversion');
end
% Passing the dimension of x1 inorder to calculate the x2 using dims2State
newRef = dims2State(dim,model2,doCov,classToUse);
end

function out = localValidateString(in)
arguments %#ok<EMFIV> 
    in {mustBeMember(in,{'constvel','constacc','constturn'})}
end
out = in;
end