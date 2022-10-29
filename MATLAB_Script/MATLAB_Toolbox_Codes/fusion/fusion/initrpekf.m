function filter = initrpekf(detection,varargin)
% INITRPEKF Constant velocity range-parameterized EKF initialization
% The range-parameterized extended Kalman filter (RPEKF) is a Gaussian-sum
% filter (trackingGSF) with multiple extended Kalman filters (EKFs), each
% initialized at an estimated range of the target. Range-parameterization
% is a commonly used technique to initialize a filter from an angle-only
% detection.
%
% filter = INITRPEKF(detection) configures the filter with 6 EKFs and the
% target range is assumed to be within 1e3 and 1e5 scenario units. The
% function configures the process noise with a unit standard deviation in
% acceleration.
%
% filter = INITRPEKF(detection, numFilters) allows specifying the
% number of EKFs in the filter.
%
% filter = INITRPEKF (detection, numFilters, rangeLimits) allows
% specifying the range limits of the target.
%
% detection must be an objectDetection with an angle-only measurement. The
% detection must specify the measurement parameters, as an array of struct
% with the following fields.
%   Frame           - either 'rectangular' or 'spherical' or an enum
%                     with the same values. The first structure must
%                     specify the Frame as 'spherical' or enum with the
%                     same value.
%   OriginPosition  - a 3-by-1 real vector.
%   OriginVelocity  - a 3-by-1 real vector.
%   Orientation     - a 3-by-3 orthonormal orientation matrix. Default: eye(3)
%   HasAzimuth      - a logical scalar, true if azimuth is measured.
%                     Default: true, must be set to true if specified as a
%                     field.
%   HasElevation    - a logical scalar, true if elevation is measured.
%                     Default: true. Can be set to false if detection
%                     contains azimuth measurement only.
%   HasVelocity     - a logical scalar, true if velocity is measured.
%                     Default: false, must be set to false if specified as
%                     a field.
%   HasRange        - a logical scalar, true if range is measured.
%                     Default: true, must be set to false and specified as
%                     a field.
%   IsParentToChild - a logical scalar, true if the orientation is given as
%                     a parent to child frame rotation.
%
% numFilters is a positive scalar integer greater than 1, defining the
% number of EKFs for range-parameterization.
%
% rangeLimits is a 2-element value defining the lower and upper limits of
% the target range.
%
% % Example 1: Initialize a RPEKF from an angle-only detection
% % -----------------------------------------------------------------------
% % Define measurement parameters
% measParam = struct('Frame','spherical','HasRange',false);
% detection = objectDetection(0,[30 30],'MeasurementNoise',0.1*eye(2),'MeasurementParameters',measParam);
% filter = INITRPEKF(detection);
%
% % Example 2: Initialize a RPEKF with 10 filters and range-limits
% % -----------------------------------------------------------------------
% % Define measurement parameters
% measParam = struct('Frame','spherical','HasRange',false);
% detection = objectDetection(0,[30 30],'MeasurementNoise',0.1*eye(2),'MeasurementParameters',measParam);
% rangeLimits = [1 1000];
% numFilters = 10;
% filter = INITRPEKF(detection, numFilters, rangeLimits);
%
% % Example 3: Using the function with trackers
% % -------------------------------------------
% funcHandle = @(detection)INITRPEKF(detection,numFilters,rangeLimits)
% tracker = trackerGNN('FilterInitializationFcn',funcHandle);
%
% See also trackingGSF, trackingEKF, objectDetection, initcvekf, 
% initcvmscekf.

% References:
% [1] Peach, N. "Bearings-only tracking using a set of range-parameterised
%   extended Kalman filters."  IEE Proceedings-Control Theory and
%   Applications 142.1 (1995): 73-80.

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
    numFilters = cast(6,classToUse);
end

if nargin > 2
    validateattributes(varargin{2},{'numeric'},{'real','finite','nonsparse','positive','increasing','vector','numel',2},funcName,'rangeLimits',3);
    Rmax = cast(varargin{2}(2),classToUse);
    Rmin = cast(varargin{2}(1),classToUse);
else
    Rmax = cast(1e5,classToUse);
    Rmin = cast(1e3,classToUse);
end

% In Simulink, during bus propagation, the detection frames are invalid.
isInvalidDet = matlabshared.tracking.internal.fusion.isInvalidDetection(detection);
if isInvalidDet
    filter = fusion.internal.zeroStateCVGSF(detection,numFilters,classToUse,true);
    return;
end

% This will fuse all the measurement parameters. Frame is defaulted to
% spherical.
[isRect,origin,velocity,orient,hasAz,hasEl,~,hasRange] = matlabshared.tracking.internal.fusion.parseDetectionForMeasurementParameters(detection,funcName,classToUse);

condInvalid = isRect;
coder.internal.errorIf(condInvalid,'fusion:GSF:invalidFlag','Frame','Spherical');

% if range is measured throw an error.
condInvalid1 = hasRange;
coder.internal.errorIf(condInvalid1,'fusion:GSF:invalidFlag','HasRange','false');

% if azimuth is not measured throw an error
condInvalid2 = ~hasAz;
coder.internal.errorIf(condInvalid2,'fusion:GSF:invalidFlag','HasAzimuth','true');

% Define range-parameterization constants.
rho = (Rmax/Rmin)^(1/numFilters);
Cr = 2*(rho - 1)/(rho + 1)/sqrt(12);

% Pad measurement with range to form a full spherical measurement.
measParam = struct('Frame','Spherical','HasVelocity',false,'HasRange',true,'HasElevation',hasEl,'OriginPosition',origin,'Orientation',orient,'OriginVelocity',velocity);
fullDet = objectDetection(detection.Time,[detection.Measurement(:);0],'MeasurementNoise',blkdiag(detection.MeasurementNoise,100),'MeasurementParameters',measParam);
    
indFilters = cell(numFilters,1);
coder.unroll();
for i = 1:numFilters
    % compute range and range-covariance for this filter.
    range = Rmin/2*(rho^i + rho^(i-1));
    rangeSigma = (range*Cr)^2;
    fullDet.Measurement(end) = range;
    fullDet.MeasurementNoise(end,end) = rangeSigma;
    % Create a filter using the dummy detection.
    dummyFilter = initcvekf(fullDet);
    thisFilter = trackingEKF(dummyFilter.StateTransitionFcn,dummyFilter.MeasurementFcn,...
        dummyFilter.State,'StateCovariance',dummyFilter.StateCovariance,...
        'HasAdditiveMeasurementNoise',dummyFilter.HasAdditiveMeasurementNoise,...
        'HasAdditiveProcessNoise',dummyFilter.HasAdditiveProcessNoise,'ProcessNoise',...
        dummyFilter.ProcessNoise,'StateTransitionJacobianFcn',dummyFilter.StateTransitionJacobianFcn,...
        'MeasurementJacobianFcn',dummyFilter.MeasurementJacobianFcn,...
        'HasMeasurementWrapping',dummyFilter.HasMeasurementWrapping);
    n = numel(detection.Measurement);
    if isscalar(detection.MeasurementNoise)
        measurementNoise = cast(matlabshared.tracking.internal.expandScalarValue(detection.MeasurementNoise, [n, n]),classToUse);
    else
        measurementNoise = cast(detection.MeasurementNoise,classToUse);
    end
    setMeasurementSizes(thisFilter,n,n);
    indFilters{i} = thisFilter;
    indFilters{i}.StateCovariance(2:2:end,2:2:end) = 100*eye(3,classToUse);
    indFilters{i}.MeasurementNoise = measurementNoise;
end

filter = trackingGSF(indFilters,ones(1,numFilters,classToUse));

end