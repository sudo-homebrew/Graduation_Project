function CKF = initcackf(Detection)
%INITCACKF  Constant acceleration trackingCKF initialization
%   CKF = INITCACKF(Detection)   initializes a constant acceleration (CA)
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
%   1. You can use INITCACKF as the FilterInitializationFcn property.
%   2. In creating the filter the function configures the process noise 
%      assuming a unit standard deviation for the acceleration change rate.
%
%   Example 1 - detection with position measurement in rectangular frame
%   --------------------------------------------------------------------
%   % The constant acceleration measurement function, cameas, provides a
%   % position measurement in 3-D. For example: x=1, y=3, z=0
%   % Use a 3-D position measurement noise [1 0.2 0; 0.2 2 0; 0 0 1];
%   detection = objectDetection(0, [1;3;0], 'MeasurementNoise', [1 0.2 0; 0.2 2 0; 0 0 1]);
% 
%   % Use INITCACKF to create a trackingCKF filter initialized at the 
%   % provided position and using the measurement noise defined above.
%   ckf1 = initcackf(detection);
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
%   % The constant acceleration measurement function, cameas, provides a
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
%   % Use INITCACKF to create a trackingCKF filter initialized at the 
%   % provided position and using the measurement noise defined above.
%   ckf2 = initcackf(detection);
%
%   % Verify that the filter state produces the same measurement as above
%   meas2 = cameas(ckf2.State, measParams)
%   isequal(meas, meas2)
%
%   See also: trackingCKF, objectDetection, initcvkf, initcakf, initcvekf,
%   initcaekf, initctekf, initcaukf, initcvukf, initctukf, constacc, cameas

%#codegen

% Copyright 2017-2018 The MathWorks, Inc.

% Input validation:
funcName = mfilename;
matlabshared.tracking.internal.fusion.validateFilterInitializationInput(Detection, funcName);

%% Convert the Measurement to Rectangular Coordinates
% A detection may have measurements in a sensor local coordinate system,
% either spherical or rectangular. This filter defines the state in a
% rectangular coordinate system relative to the ego vehicle. This requires
% the conversion of the measurement between frames
classToUse = class(Detection.Measurement);
[posMeas, velMeas, posCov, velCov, invalidDet] = ...
    matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(Detection, funcName, classToUse);

%% How to Initialize a Tracking Filter?
% Things to consider when creating a filter:
%   1. Defining the motion model and state
%   2. Defining the process noise
%   3. Defining the measurement model
%   4. Mapping the sensor measurements to an initial state vector
%   5. Mapping the sensor measurement noise to a state covariance
%   6. Creating the correct filter

%% Step 1: Defining the motion model and state
%   In this example, use a constant acceleration model in a rectangular
%   frame
%       1. The state is [x;vx;ax;y;vy;ay;z;vz;az]
%       2. The state transition function is constacc
 
%% Step 2: Defining the process noise 
%   The process noise represents the parts of the process that are not
%   taken into account in the model. For example, in a constant
%   acceleration model, the change in the acceleration is neglected. To
%   account for it, a scaled 1-d rate of acceleration change represents the
%   magnitude of the possible change in the acceleration that is neglected
%   in each direction.
scaleAccelChange = ones(1, classToUse);
Q = blkdiag(scaleAccelChange^2, scaleAccelChange^2, scaleAccelChange^2);

%% Step 3: Defining the measurement model
%   Regardless of the motion dimensionality or coordinates frame, cameas
%   can be used to produce the measurement.
%       1. The measurement function is cameas.
 
%% Step 4: Initializing the state vector using the Measurement
%   Since the state is [x;vx;ax;y;vy;ay;z;vz;az], but the acceleration is
%   not measured, we initialize the acceleration components to zero.
H1d = cast([1 0 0], classToUse);
Hpos = blkdiag(H1d, H1d, H1d);                  % position = Hpos * state
Hvel = [zeros(3,1,classToUse),Hpos(:,1:end-1)]; % velocity = Hvel * state
state = Hpos' * posMeas(:) + Hvel'*velMeas(:);

%% Step 5: Initializing the state covariance using MeasurementNoise
%   For the parts of the state that are measured directly by the sensor,
%   use the corresponding measurement noise components. For the parts that
%   are not measured directly, assume a large initial state covariance.
%   This will allow future detections to be assigned to the track.
L = 100; % A large value compared to the expected measurement noise.
accCov = cast(diag([0 0 L 0 0 L 0 0 L]),classToUse); % Covariance corresponding to acceleration elements
stateCov = Hpos'*posCov*Hpos + Hvel'*velCov*Hvel + accCov;
 
%% Step 6: Creating the correct filter
%   Creating a trackingCKF filter with a constant acceleration model and
%   the properties defined above
if invalidDet % Do not set measurement noise if the detection is invalid
    CKF = trackingCKF(@constacc, @cameas, state, ...
        'StateCovariance', stateCov, ...
        'HasAdditiveProcessNoise', false, ...
        'HasMeasurementWrapping', true, ...
        'ProcessNoise', Q ...
        );
else
    n = numel(Detection.Measurement);
    if isscalar(Detection.MeasurementNoise)
        measurementNoise = cast(matlabshared.tracking.internal.expandScalarValue(Detection.MeasurementNoise, [n, n]),classToUse);
    else
        measurementNoise = cast(Detection.MeasurementNoise,classToUse);
    end
    CKF = trackingCKF(@constacc, @cameas, state, ...
        'StateCovariance', stateCov, ...
        'MeasurementNoise', measurementNoise, ...
        'HasAdditiveProcessNoise', false, ...
        'HasMeasurementWrapping', true, ...
        'ProcessNoise', Q ...
        );
    setMeasurementSizes(CKF, n, n); % Helps coder deduce the measurement sizes
end