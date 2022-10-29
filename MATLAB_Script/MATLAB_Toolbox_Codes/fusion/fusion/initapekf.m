function filter = initapekf(detection,varargin)
% INITAPEKF Constant velocity angle-parameterized EKF initialization
% The angle-parameterized extended Kalman filter(APEKF) is a Gaussian-sum
% filter (trackingGSF) with multiple extended Kalman filters (EKFs), each
% initialized at an estimated angular position of the target.
% Angle-parametrization is a commonly used technique to initialize a filter
% from a range-only detection.
%
% filter = INITAPEKF(detection) configures the filter with 10 EKFs and the
% angular limits are assumed to be within [-180 180] degrees. The function
% configures the process noise with unit standard deviation in acceleration.
%
% filter = INITAPEKF(detection, numFilters) allows specifying the
% number of EKFs in the filter.
%
% filter = INITAPEKF(detection, numFilters, angleLimits) allows
% specifying the limits on angular position of the target.
%
% detection must be an objectDetection and must specify the measurement 
% parameters, as a scalar or an array of struct with the following fields.
%       Frame          - either 'rectangular' or 'spherical' or an enum
%                        with the same values. The first structure must
%                        specify the Frame as 'spherical' or enum with the
%                        same value.
%       OriginPosition - a 3-by-1 real vector.
%       OriginVelocity - a 3-by-1 real vector.
%       Orientation    - a 3-by-3 orthonormal orientation matrix.
%       HasAzimuth     - a logical scalar, true if azimuth is measured.
%                        Default: true. Can be set to false if detection
%                        does not contain azimuth angle.
%       HasElevation   - a logical scalar, true if elevation is measured.
%                        Default: true. Can be set to false if detection
%                        does not contain elevation angle.
%       HasVelocity    - a logical scalar, true if velocity is measured.
%                        Default: true. Can be set to false if detection
%                        does not contain range-rate.
%       HasRange       - a logical scalar, true if range is measured.
%                        Default: true, must be set to true if specified
%                        as a field.
%       IsParentToChild - a logical scalar, true if the orientation is
%                         given as a parent to child frame rotation.
%
% The function can support the following types of measurements in the
% detection.
% 1. range measurements - parameterization is done on azimuth of the
%    target.
% 2. azimuth and range measurements - parameterization is done on
%    elevation of the target.
%
% numFilters is a positive scalar integer greater than 1, defining the
% number of EKFs for angle-parameterization.
%
% angleLimits is a 2-element value defining the lower and upper limits of
% the target angular position.
%
% % Example 1: Initialize a angle-parameterized EKF from an range-only detection
% % -------------------------------------------------------------------------
% % create range-only measurement parameters
% measParam = struct('Frame','Spherical','HasAzimuth',false,'HasElevation',false,'HasVelocity',false);
% detection = objectDetection(0,1000,'MeasurementNoise',100,'MeasurementParameters',measParam);
% filter = INITAPEKF(detection);
%
% % Example 2: Initialize a angle-parameterized EKF from a range-only detection
% % with 10 filters, angle limits as -30 to 30 degrees.
% % -------------------------------------------------------------------------
% % create range-only measurement parameters
% measParam = struct('Frame','Spherical','HasAzimuth',false,'HasElevation',false,'HasVelocity',false);
% detection = objectDetection(0,1000,'MeasurementNoise',100,'MeasurementParameters',measParam);
% angleLimits = [-30 30];
% numFilters = 10;
% filter = INITAPEKF(detection, numFilters, angleLimits);
%
% % Example 3: Using the function with trackers
% % -------------------------------------------------------------------------
% funcHandle = @(detection)INITAPEKF(detection,numFilters,angleLimits)
% tracker = trackerGNN('FilterInitializationFcn',funcHandle);
%
% See also trackingGSF, trackingEKF, objectDetection, initcvekf.

% Copyright 2018 The MathWorks, Inc.

%#codegen

% validate detection input
funcName = mfilename;
matlabshared.tracking.internal.fusion.validateFilterInitializationInput(detection,funcName);

classToUse = class(detection.Measurement);

if nargin  > 1 
    validateattributes(varargin{1},{'numeric'},{'real','finite','nonsparse','positive','integer','scalar'},funcName,'numFilters',2);
    numFilters = cast(varargin{1},classToUse);
else
    numFilters = cast(10,classToUse);
end

if nargin > 2
    validateattributes(varargin{2},{'numeric'},{'real','finite','nonsparse','increasing','vector','numel',2},funcName,'angleLimits',3);
    angleMin = cast(varargin{2}(1),classToUse);
    angleMax = cast(varargin{2}(2),classToUse);
    % Check if the limits were provided or were default.
    isDefault = false;
else
    angleMin = cast(-180,classToUse);
    angleMax = cast(180,classToUse);
    isDefault = true;
end

% In Simulink, during bus propagation, the detection frames are invalid.
isInvalidDet =  matlabshared.tracking.internal.fusion.isInvalidDetection(detection);
if isInvalidDet
    filter = fusion.internal.zeroStateCVGSF(detection,numFilters,classToUse);
    return;
end

% This will fuse all the measurement parameters. Frame is defaulted to
% spherical.
[isRect,origin,velocity,orient,hasAz,hasEl,hasVel,hasRange] = matlabshared.tracking.internal.fusion.parseDetectionForMeasurementParameters(detection,funcName,classToUse);

% Frame is rectangular
condInvalid = isRect;
coder.internal.errorIf(condInvalid,'fusion:GSF:invalidFlag','Frame','Spherical');

% if both angles are available, throw an error.
condInvalid1 = hasAz && hasEl;
coder.internal.errorIf(condInvalid1,'fusion:GSF:invalidCombinationFlag','HasElevation','HasAzimuth','false');

% Change elevation limits = [-90 90], if azimuth was available.
if hasAz && isDefault
    angleMax = angleMax/2;
    angleMin = angleMin/2;
end

% if range is not measured, throw an error.
condInvalid2 = ~hasRange;
coder.internal.errorIf(condInvalid2,'fusion:GSF:invalidFlag','HasRange','true');

expMeasSize = matlabshared.tracking.internal.fusion.getExpectedMeasurementSize(isRect,hasAz,hasEl,hasVel,hasRange);
validateattributes(detection.Measurement,{'numeric'},{'numel',expMeasSize},funcName,'Detection.Measurement');

% Create a spherical measurement frame for using initcvekf.
measParam = struct('Frame','Spherical','HasVelocity',true,'HasElevation',true,'HasAzimuth',true,'OriginPosition',origin,'Orientation',orient,'OriginVelocity',velocity);
dummyMeasSize = 4; % full spherical measurement
fullDet = objectDetection(detection.Time,zeros(dummyMeasSize,1,classToUse),'MeasurementNoise',zeros(dummyMeasSize,classToUse),'MeasurementParameters',measParam);

indFilters = cell(numFilters,1);
n = numel(detection.Measurement);
meas = zeros(4,1,classToUse);
measCov = zeros(4,classToUse);

for i = 1:numFilters
    angle = angleMin + (angleMax - angleMin)/numFilters*(i - 0.5);
    angleSigma = ((angleMax - angleMin)/(sqrt(12)*numFilters))^2;
    if n == 1 %[r] only
        meas(1) = angle;
        measCov(1,1) = angleSigma;
        meas(2) = cast(0,classToUse);
        measCov(2,2) = cast(0.01,classToUse);
        meas(3) = detection.Measurement(1);
        measCov(3,3) = detection.MeasurementNoise(1);
        meas(4) = cast(0,classToUse);
        measCov(4,4) = cast(100,classToUse);
    elseif n == 2 % [az r] or [r (rr)]
        if hasVel %[r (rr)]
            meas(1) = angle;
            meas(2) = cast(0,classToUse);
            meas(3:end) = detection.Measurement(1:end);
            measCov(:) = blkdiag(angleSigma,cast(0.01,classToUse),detection.MeasurementNoise(1:end,1:end));
        else % [az r]
            meas(1) = detection.Measurement(1);
            meas(2) = angle;
            meas(3) = detection.Measurement(2);
            meas(4) = 0;
            % Don't destroy cross covariance
            measCov(:) = blkdiag(detection.MeasurementNoise(1,1),angleSigma,detection.MeasurementNoise(2,2),cast(100,classToUse));
            measCov(1,3) = detection.MeasurementNoise(1,2);
            measCov(3,1) = detection.MeasurementNoise(2,1);
        end
    else % [az r (rr)]
        meas(:) = [detection.Measurement(1);angle;detection.Measurement(2:end)];
        measCov([1 3 4],[1 3 4]) = detection.MeasurementNoise;
        measCov(2,2) = angleSigma;
    end
    fullDet.Measurement = meas;
    fullDet.MeasurementNoise = measCov;
    dummyFilter = initcvekf(fullDet);
    thisFilter = trackingEKF(dummyFilter.StateTransitionFcn,dummyFilter.MeasurementFcn,...
        dummyFilter.State,'StateCovariance',dummyFilter.StateCovariance,...
        'HasAdditiveMeasurementNoise',dummyFilter.HasAdditiveMeasurementNoise,...
        'HasAdditiveProcessNoise',dummyFilter.HasAdditiveProcessNoise,'ProcessNoise',...
        dummyFilter.ProcessNoise,'StateTransitionJacobianFcn',dummyFilter.StateTransitionJacobianFcn,...
        'MeasurementJacobianFcn',dummyFilter.MeasurementJacobianFcn);
    n = numel(detection.Measurement);
    if isscalar(detection.MeasurementNoise)
        measurementNoise = cast(matlabshared.tracking.internal.expandScalarValue(detection.MeasurementNoise, [n, n]),classToUse);
    else
        measurementNoise = cast(detection.MeasurementNoise,classToUse);
    end
    setMeasurementSizes(thisFilter,n,n);
    indFilters{i} = thisFilter;
    indFilters{i}.MeasurementNoise = measurementNoise;
end

filter = trackingGSF(indFilters,ones(1,numFilters,classToUse));

end