%INITCVCKF Constant velocity trackingCKF initialization
%   CKF = INITCVCKF(Detection)   initializes a constant velocity (CV)
%   trackingCKF (CKF) based on information provided in Detection. Detection
%   must be an objectDetection. The detection must specify the measurement
%   parameters, as a struct with the following fields. Default values are
%   used when a field is missing:
%     Frame           - 'rectangular' or 'spherical'. Default: 'rectangular'
%     OriginPosition  - a 3-by-1 real vector.  Default: [0;0;0]
%     OriginVelocity  - a 3-by-1 real vector.  Default: [0;0;0]
%     Orientation     - a 3-by-3 orthonormal 
%                       orientation matrix.    Default: eye(3)
%     HasElevation    - a logical scalar.      Default: true, elevation is measured
%     HasVelocity     - a logical scalar.      Default: false in rectangular, true in spherical
%     IsParentToChild - a logical scalar.      Default: false, orientation defines a rotation from child to parent frame
%
%   Notes:
%   ------
%   1. You can use INITCVCKF as the FilterInitializationFcn property.
%   2. In creating the filter the function configures the process noise 
%      assuming a unit acceleration standard deviation.
%
%   Example 1 - detection with position measurement in rectangular frame
%   --------------------------------------------------------------------
%   % The constant velocity measurement function, cvmeas, provides a
%   % position measurement in 3-D. For example: x=1, y=3, z=0.
%   % Use a 3-D position measurement noise [1 0.2 0; 0.2 2 0; 0 0 1];
%   detection = objectDetection(0, [1;3;0], 'MeasurementNoise', [1 0.2 0; 0.2 2 0; 0 0 1]);
% 
%   % Use INITCVCKF to create a trackingCKF filter initialized at the 
%   % provided position and using the measurement noise defined above.
%   ckf1 = initcvckf(detection);
%
%   % Check the values of the state and measurement noise. 
%   % Verify that the filter state, ckf1.State, has the same position 
%   % components as detection measurement, detection.Measurement.  Verify 
%   % that the filter measurement noise, ckf1.MeasurementNoise, is the same 
%   % as the detection.MeasurementNoise values. 
%   ckf1.State 
%   ckf1.MeasurementNoise
%
%   Example 2 - detection with position measurement in spherical frame
%   ------------------------------------------------------------------
%   % The constant velocity measurement function, cvmeas, provides a
%   % spherical measurement. For example: az = 30, el = 5, r = 100, rr = 4
%   % Measurement noise is diag([2.5, 2.5, 0.5, 1].^2)
%   meas = [30;5;100;4];
%   measNoise = diag([2.5, 2.5, 0.5, 1].^2);
%
%   % Use the MeasurementParameters to define the frame. You can leave out
%   % other fields of the MeasurementParameters struct and they will be
%   % completed by default values. In this example, sensor position, sensor
%   % velocity, orientation, elevation and range rate flags are default.
%   measParams = struct('Frame','spherical');
%   detection = objectDetection(0, meas, 'MeasurementNoise', measNoise,...
%       'MeasurementParameters', measParams);
% 
%   % Use INITCVCKF to create a trackingCKF filter initialized at the 
%   % provided position and using the measurement noise defined above.
%   ckf2 = initcvckf(detection);
%
%   % Verify that the filter state produces the same measurement as above
%   meas2 = cvmeas(ckf2.State, measParams)
%   isequal(meas, meas2)
%
%   See also: trackingCKF, objectDetection, initcvkf, initcakf, initcvekf,
%   initcaekf, initctekf, initcaukf, initcvukf, initctukf, initcackf,
%   initctckf, constvel, cvmeas

 
% Copyright 2017-2018 The MathWorks, Inc.
