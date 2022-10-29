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

 
% Copyright 2018 The MathWorks, Inc.

