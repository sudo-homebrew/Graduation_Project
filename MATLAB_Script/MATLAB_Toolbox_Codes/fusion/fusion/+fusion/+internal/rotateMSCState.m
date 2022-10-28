function [mscState,rotJacobian] = rotateMSCState(mscState,rotMatrix)
%This is an internal function and may be remove in future.
% rotateMSCState - rotate a state in MSC frame using a rotation matrix
% specified in scenario frame.
% Inputs:
%   mscState - state in MSC frame.
%   rotMatrix - a 3x3 matrix defining rotation 
% Outputs:
%   mscState - rotated state in MSC frame.
%   rotJacobian - Jacobian of rotation transformation.

% Copyright 2018 The MathWorks, Inc.


%#codegen

% No validation is performed in this function. Validation should be
% performed at the caller levels.

% Don't do anything if rotMatrix is identity.
if ~isequal(rotMatrix,eye(3,class(rotMatrix)))
    % Convert msc to cartesian frame.
    cartState = fusion.internal.mscToCartesian(mscState);
    % mscToCartesianGradient must be calculated at un-rotated state.
    J2 = fusion.internal.mscToCartesianGradient(mscState);
        
    % Rotate cartesian state using rotation matrix.
    orientTotal = zeros(6,'like',mscState);
    orientTotal(1:2:end,1:2:end) = rotMatrix;
    orientTotal(2:2:end,2:2:end) = rotMatrix;
    cartState = orientTotal*cartState;
    
    % Transform back to msc frame.
    mscState = fusion.internal.cartesianToMSC(cartState);
    
    % return jacobian of the rotation, only if demanded.
    if nargout == 2
        % cartesianToMSCGradient must be calculated at rotated state.
        J1 = fusion.internal.cartesianToMSCGradient(cartState);
        rotJacobian = J1*orientTotal*J2;
    end
else
    if nargout == 2
        rotJacobian = eye(6,'like',mscState); 
    end
end