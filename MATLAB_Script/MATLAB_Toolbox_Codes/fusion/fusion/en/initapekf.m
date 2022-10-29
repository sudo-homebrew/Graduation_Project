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

