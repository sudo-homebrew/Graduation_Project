function [meas, wrapping] = cvmeasmsc(state,varargin)
% CVMEASMSC Measurement based on constant velocity (CV) model in MSC frame
%   Tracking filters require a definition of MeasurementFcn
%   property. CVMEASMSC provides the measurement from state used in the
%   motion model, constvelmsc.
%
%   measurement = CVMEASMSC(state) provides the angular measurement
%   (azimuth and elevation) of the state in the sensor frame described by
%   the mscState. The state can be either in 2-D or 3-D. 
%       In a 2-D case: state = [az azRate 1/r vr/r];
%       In a 3-D case: state = [az omega el elRate 1/r vr/r];
%   where:
%   az = azimuth angle (rad)
%   el = elevation angle (rad)
%   azRate = azimuth rate (rad/s)
%   elRate = elevation rate (rad/s)
%   omega = azRate * cos(el) (rad/s)
%   1/r = 1/range (1/m)
%   vr/r = range-rate/range or inverse time-to-go. (1/s)
%
%   If the input, state, is specified as a matrix, states must be
%   concatenated along columns, where each column represents a state
%   following the convention specified above. The output will be a matrix 
%   with the same number of columns as the input, where each column 
%   represents the measurement from the corresponding state.
%
%   If motion model is in 2-D, values corresponding to elevation are 
%   assumed to be zero if elevation is requested as an output.
%
%   measurement = CVMEASMSC(state,frame) provides the measurement in the
%   frame specified. The allowed values for frame are 'rectangular' and
%   'spherical' (default)
%   If using 'rectangular' frame, the three elements present in the
%   measurement represent the x, y and z position of the target in the
%   observer's Cartesian frame.
%
%   measurement = CVMEASMSC(state,frame,laxes) specifies the axes, in
%   LAXES, of the local coordinate system. LAXES is a 3-by-3 orthonormal
%   orientation matrix whose columns specify the directions of the local x,
%   y, and z axes in the global Cartesian frame.
%
%   measurement = CVMEASMSC(state,measurementParameters) specifies the
%   measurement parameters as a scalar struct or an array of struct. The 
%   structures must have the following fields (or a subset of them).
%       Frame           - either 'rectangular' or 'spherical' or an enum
%                         with the same values. Default: 'spherical'
%       Orientation     - a 3-by-3 orthonormal orientation matrix. Default:
%                         eye(3).
%       HasElevation    - a logical scalar, true if elevation is measured.
%                         Default: true if state is 3D, false if state is
%                         2D
%       IsParentToChild - a logical scalar, true if the orientation is
%                         given as a parent to child frame rotation.
%
%   [measurement, bounds] = CVMEASMSC(...), additionally, returns the
%   measurement bounds, which are used by the filter in residual
%   calculations. The function returns the following bounds based on
%   the frame.
%       Rectangular: An 3-by-2 matrix where 3 is the number of elements in
%                    the measurement and each row is [-Inf Inf], indicating
%                    no wrapping.
%       Spherical:   An M-by-2 matrix where M is the number of elements in
%                    the measurement. The rows are in the following order
%                       Azimuth bounds are always the first row: [-180 180]
%                       When HasElevation = true, second row is: [-90 90]
%
%   % Example 1: Obtain azimuth and elevation measurement from a MSC state.
%   % ---------------------------------------------------------------------
%   mscState = [0.5;0;0.3;0;1e-3;1e-2];
%   cvmeasmsc(mscState)
%
%   % Example 2: Obtain position measurement from a MSC state.
%   % --------------------------------------------------------
%   mscState = [0.5;0;0.3;0;1e-3;1e-2];
%   % Specify the frame as a second input.
%   cvmeasmsc(mscState,'rectangular')
%   % Specifying the frame using measurementParameters
%   cvmeasmsc(mscState,struct('Frame','rectangular'));
%
%   See also trackingMSCEKF, initcvmscekf, constvelmsc, cvmeasmscjac

%   Copyright 2018-2021 The MathWorks, Inc.

%#codegen

narginchk(1,3);

% Validate state input
validateattributes(state, {'single','double'},...
    {'real', 'finite', '2d', 'nonsparse'}, 'cvmeasmsc', 'state', 1);

% Check for row or column Orientation
[rowSize,colSize] = size(state);
isRowOrient = (colSize == 4) || (colSize == 6);
isColOrient = (rowSize == 4) || (rowSize == 6);
classToUse = class(state);

condSize = (~isRowOrient || ~isvector(state)) && ~isColOrient;
coder.internal.errorIf(condSize,'shared_tracking:motion:incorrectStateSizeWithInfo','[4 6]');

if ~isColOrient
    stateCol = state';
else
    stateCol = state;
end

hasZStates = size(stateCol,1) == 6;

if ~hasZStates
    zeroState = zeros(1,size(stateCol,2),'like',stateCol);
    fullStateCol = [stateCol(1,:);stateCol(2,:);zeroState;zeroState;stateCol(3,:);stateCol(4,:)];
else
    fullStateCol = stateCol;
end

[isRect,orient,hasEl] = fusion.internal.parseMeasurementParametersForMSC(hasZStates,mfilename,classToUse,varargin{:});

% rotation should be transformed as this is global to local coordinate
% conversion.
fullStateCol = fusion.internal.rotateMSCState(fullStateCol,orient');

if ~isRect
    measSize = 1 + hasEl;
    meas = zeros(measSize,size(stateCol,2),classToUse);
    meas(1,:) = rad2deg(fullStateCol(1,:));
    if hasEl
        meas(2,:) = rad2deg(fullStateCol(3,:));
    end
else
    cartState = fusion.internal.mscToCartesian(fullStateCol);
    measSize = 3;
    meas = cartState(1:2:end,:);
end

wrapping = zeros(measSize,2,classToUse);
if isRect
    wrapping(:) = matlabshared.tracking.internal.defaultWrapping(cast(3,classToUse),classToUse);
else
    wrapping(1,:) = [-180 180];
    if hasEl
        wrapping(2,:) = [-90 90];
    end
end
meas = matlabshared.tracking.internal.wrapResidual(meas,wrapping,'cvmeasmsc');
end