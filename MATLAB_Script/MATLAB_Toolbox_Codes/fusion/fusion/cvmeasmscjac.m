function jac = cvmeasmscjac(state,varargin)
% CVMEASMSCJAC  Jacobian of measurement using constant velocity (CV) model in MSC frame
%   The trackingEKF object allows you to specify the MeasurementJacobianFcn
%   property. CVMEASMSCJAC provides the Jacobian of the constant velocity
%   measurement function, cvmeasmsc, with respect to the state.
%
%   jacobian = CVMEASMSCJAC(state) calculates the jacobian with respect to
%   angular measurement (azimuth and elevation) of the state in the sensor
%   frame described by. The motion can be either in 2-D or 3-D.
%   If motion model is in 2D, values corresponding to elevation are assumed
%   to be zero.
%
%   jacobian = CVMEASMSCJAC(state,frame) provides the measurement jacobian
%   in the frame specified. The allowed values for frame are 'rectangular'
%   and 'spherical' (default)
%   If using 'rectangular' frame, the three elements present in the
%   measurement represent the x, y and z position of the target in the
%   observer's Cartesian frame.
%
%   jacobian = CVMEASMSCJAC(state,frame,laxes) specifies the axes, in
%   LAXES, of the local coordinate system. LAXES is a 3-by-3 orthonormal
%   orientation matrix whose columns specify the directions of the local x,
%   y, and z axes in the global Cartesian frame.
%
%   jacobian = CVMEASMSC(state,measurementParameters) specifies the
%   measurement parameters as a struct. The struct must have the following
%   fields (or a subset of them).
%       Frame           - either 'rectangular' or 'spherical' or an enum
%                         with the same values. Default: 'spherical'
%       Orientation     - a 3-by-3 orthonormal orientation matrix. Default:
%                         eye(3).
%       HasElevation    - a logical scalar, true if elevation is measured.
%       IsParentToChild - a logical scalar, true if the orientation is
%                         given as a parent to child frame rotation.
%
%   % Example 1: Obtain azimuth and elevation measurement jacobian
%   % ------------------------------------------------------------
%   mscState = [0.5;0;0.3;0;1e-3;1e-2];
%   cvmeasmscjac(mscState)
%
%   % Example 2: Obtain position measurement jacobian
%   % -----------------------------------------------
%   mscState = [0.5;0;0.3;0;1e-3;1e-2];
%   % Specify the frame as second input.
%   cvmeasmscjac(mscState,'rectangular');
%   % Specify the frame using measurementParameters
%   cvmeasmscjac(mscState,struct('Frame','rectangular'));
%
%   See also trackingMSCEKF, initcvmscekf, constvelmsc, cvmeasmsc

%   Copyright 2018 The MathWorks, Inc.

%#codegen
% Validate state input

validateattributes(state, {'single','double'},...
    {'real', 'finite', 'vector', 'nonsparse'}, 'cvmeasmscjac', 'state', 1);

% Check for row or column Orientation
[rowSize,colSize] = size(state);
isRowOrient = (colSize == 4) || (colSize == 6);
isColOrient = (rowSize == 4) || (rowSize == 6);

cond = (~isRowOrient || isvector(state)) && ~isColOrient;
coder.internal.errorIf(cond,'shared_tracking:motion:incorrectStateVecWithInfo','[4 6]');

hasZStates = numel(state) == 6;
stateDim = numel(state)/2;

if ~hasZStates
    zeroState = zeros(1,1,'like',state);
    stateCol = [state(1);state(2);zeroState;zeroState;state(3);state(4)];
else
    stateCol = state(:);
end

classToUse = class(state);

[isRect,orient,hasEl] = fusion.internal.parseMeasurementParametersForMSC(hasZStates,mfilename,classToUse,varargin{:});

[~,rotJacobian] = fusion.internal.rotateMSCState(stateCol,orient');

radDegFactor = cast(57.295779513082323,'like',stateCol);

if ~isRect
    measSize = 1 + hasEl;
    jac = zeros(measSize,2*stateDim,'like',stateCol);
    jac(1,1) = radDegFactor;
    if hasEl && hasZStates
        jac(2,3) = radDegFactor;
    end
else
    mscToCartJac = fusion.internal.mscToCartesianGradient(stateCol);
    if ~hasZStates
        jac = mscToCartJac(1:2:end,[1 2 5 6]);
    else
        jac = mscToCartJac(1:2:end,:);
    end
end

if hasZStates
    jac = jac*rotJacobian;
else
    jac = jac*rotJacobian([1 2 5 6],[1 2 5 6]);
end

end