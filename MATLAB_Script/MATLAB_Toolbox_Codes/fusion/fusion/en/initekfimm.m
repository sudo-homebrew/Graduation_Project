%INITEKFIMM  trackingIMM initialization
%   IMM = INITEKFIMM(Detection) initializes a constant velocity (CV),
%   constant acceleration (CA) and constant turn (CT) trackingIMM (IMM)
%   based on information provided in Detection.
%   Detection must be an objectDetection. The detection must specify the
%   measurement parameters, as a struct with the following fields.
%   Default values are used when a field is missing:
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
%   1. You can use initekfimm as the FilterInitializationFcn property.
%
%   Example 1 - detection with position measurement in rectangular frame
%   --------------------------------------------------------------------
%   % A 3-D position measurement in rectangular frame is provided.
%   % For example: x = 1, y = 3, z = 0.
%   % Use a 3-D position measurement noise [1 0.4 0; 0.4 4 0; 0 0 1];
%   detection = objectDetection(0, [1;3;0], 'MeasurementNoise', [1 0.4 0; 0.4 4 0; 0 0 1]);
%
%   % Use initekfimm to create a trackingIMM filter initialized at the
%   % provided position and using the measurement noise defined above.
%   imm = initekfimm(detection);
%
%   % Check the values of the state and measurement noise.
%   % Verify that the filter state, imm.State, has the same position
%   % components as detection measurement, detection.Measurement.  Verify
%   % that the filter measurement noise, imm.MeasurementNoise, is the same
%   % as the detection.MeasurementNoise values.
%   imm.State
%   imm.MeasurementNoise
%
%   Example 2 - detection with position measurement in spherical frame
%   ------------------------------------------------------------------
%   % A 3-D position measurement in spherical frame is provided.
%   % For example: az = 40, el = 6, r = 100, rr = 5
%   % Measurement noise is diag([2.5, 2.5, 0.5, 1].^2)
%   meas = [40;6;100;5];
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
%   % Use initekfimm to create a trackingIMM filter initialized at the
%   % provided position and using the measurement noise defined above.
%   imm = initekfimm(detection);
%
%   See also: trackingIMM, objectDetection, initcvekf, initcaekf, initctekf

 
% Copyright 2018-2019 The MathWorks, Inc.

