function EKF = initsingerekf(Detection)
%initsingerekf  Singer acceleration trackingEKF initialization
%   EKF = initsingerekf(Detection)   initializes a Singer acceleration (CA)
%   trackingEKF (EKF) based on information provided in Detection. Detection
%   must be an objectDetection object. The detection must specify the
%   measurement parameters as a struct with the following fields. Default
%   values are used when a field is missing:
%     Frame			  - 'rectangular' or 'spherical'. Default: 'rectangular'
%     OriginPosition  - a 3-by-1 real vector.  Default: [0;0;0]
%     OriginVelocity  - a 3-by-1 real vector.  Default: [0;0;0]
%     Orientation     - a 3-by-3 orthonormal 
%                       orientation matrix.    Default: eye(3)
%     HasRange        - a logical scalar.      Default: true, range is measured
%     HasAzimuth      - a logical scalar.      Default: true, azimuth is measured
%     HasElevation    - a logical scalar.      Default: true, elevation is measured
%     HasVelocity     - a logical scalar.	   Default: false in rectangular, true in spherical
%	  IsParentToChild - a logical scalar.	   Default: false, orientation defines a rotation from child to parent frame
%
%   Notes:
%   ------
%   1. You can use initsingerekf as the FilterInitializationFcn property.
%   2. In creating the filter, the function configures the process noise
%      assuming a unit target maneuver time constant, tau, and a unit
%      target maneuver standard deviation, sigmam. The function uses the
%      singerProcessNoise function in this step.
%   3. The Singer process noise assumes an invariant time step and additive
%      process noise.
%
%   Example 1 - detection with position measurement in rectangular frame
%   --------------------------------------------------------------------
%   % The acceleration measurement function, singermeas, provides a
%   % position measurement in 3-D. For example: x=1, y=3, z=0
%   % Use a 3-D position measurement noise [1 0.2 0; 0.2 2 0; 0 0 1];
%   detection = objectDetection(0, [1;3;0], 'MeasurementNoise', [1 0.2 0; 0.2 2 0; 0 0 1]);
% 
%   % Use initsingerekf to create a trackingEKF filter initialized at the 
%   % provided position and using the measurement noise defined above.
%   ekf1 = initsingerekf(detection);
%
%   % Check the values of the state and measurement noise. 
%   % Verify that the filter state, ekf1.State, has the same position 
%   % components as detection measurement, detection.Measurement.  Verify 
%   % that the filter measurement noise, ekf1.MeasurementNoise, is the same 
%   % as the detection.MeasurementNoise values. 
%   ekf1.State 
%   ekf1.MeasurementNoise
%
%   Example 2 - detection with position measurement in spherical frame
%   ------------------------------------------------------------------
%   % The acceleration measurement function, singermeas, provides a
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
%   % Use initsingerekf to create a trackingEKF filter initialized at the 
%   % provided position and using the measurement noise defined above.
%   ekf2 = initsingerekf(detection);
%
%   % Verify that the filter state produces the same measurement up to
%   % roundoff error
%   meas2 = singermeas(ekf2.State, measParams)
%   max(abs(meas-meas2))
%
%   See also: trackingEKF, objectDetection, singer, singerjac,
%   singerProcessNoise, initcvkf, initcakf, initcvekf, initcaekf,
%   initctekf, initcvukf, initcaukf, initctukf, singermeas, singermeasjac

%#codegen

% Copyright 2020-2021 The MathWorks, Inc.

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
%   In this example, use a Singer acceleration model in a rectangular
%   frame
%       1. The state is [x;vx;ax;y;vy;ay;z;vz;az]
%       2. The state transition function is singer
%       3. The state transition Jacobian function is singerjac

%% Step 2: Initializing the state vector using the Measurement
%   Since the state is [x;vx;ax;y;vy;ay;z;vz;az], but the acceleration is
%   not measured, we initialize the acceleration components to zero.
H1d = cast([1 0 0], classToUse);
Hpos = blkdiag(H1d, H1d, H1d);                  % position = Hpos * state
Hvel = [zeros(3,1,classToUse),Hpos(:,1:end-1)]; % velocity = Hvel * state
state = Hpos' * posMeas(:) + Hvel'*velMeas(:);

%% Step 3: Defining the process noise 
%   The process noise represents the parts of the process that are not
%   taken into account in the model. For the Singer acceleration model, the
%   process noise is defined as additive and is calculated based on the
%   parameters T (timestep), tau (target maneuver time constant), and
%   sigmam (target maneuver standard deviation). The process noise is
%   assumed to be time invariant, in other words the timestep is assumed
%   constant.
%   We are assuming that the target maneuver time constant, taue, in all 3
%   dimensions is [20;20;20]. This is also used when setting up the
%   StateTransitionFcn and StateTransitionJacobianFcn.
T = 1;            % 1 second time step
sigmam = [1;1;1]; % Target maneuver standard deviation in all 3 dimensions
Q = singerProcessNoise(ones(9,1), T, [20;20;20], sigmam);
 
%% Step 4: Defining the measurement model
%   Regardless of the motion dimensionality or coordinates frame, singermeas
%   can be used to produce the measurement.
%       1. The measurement function is singermeas.
%       2. The measurement Jacobian function is singermeasjac.
 
%% Step 5: Initializing the state covariance using MeasurementNoise
%   For the parts of the state that are measured directly by the sensor,
%   use the corresponding measurement noise components. For the parts that
%   are not measured directly, assume a large initial state covariance.
%   This will allow future detections to be assigned to the track.
L = 100; % A large value compared to the expected measurement noise.
accCov = cast(diag([0 0 L 0 0 L 0 0 L]),classToUse); % Covariance corresponding to acceleration elements
stateCov = Hpos'*posCov*Hpos + Hvel'*velCov*Hvel + accCov;
 

% Even though we defined tau above, code generation cannot lock tau as a
% constant value in this function. Therefore, we have to define it here
% explicitly for the anonymous functions
f = @(state,dt) singer(state,dt,[20;20;20]);
dfdx = @(state,dt) singerjac(state,dt,[20;20;20]);

%% Step 6: Creating the correct filter
%   Creating a trackingEKF filter with a Singer acceleration model and
%   the properties defined above
if invalidDet % Do not set measurement noise if the detection is invalid
    EKF = trackingEKF(f, @singermeas, state, ...
        'StateTransitionJacobianFcn', dfdx, ...
        'MeasurementJacobianFcn', @singermeasjac, ...
        'HasMeasurementWrapping', true, ...
        'StateCovariance', stateCov, ...
        'HasAdditiveProcessNoise', true, ...
        'ProcessNoise', Q ...
        );
else
    n = numel(Detection.Measurement);
    if isscalar(Detection.MeasurementNoise)
        measurementNoise = cast(matlabshared.tracking.internal.expandScalarValue(Detection.MeasurementNoise, [n, n]),classToUse);
    else
        measurementNoise = cast(Detection.MeasurementNoise,classToUse);
    end
    EKF = trackingEKF(f, @singermeas, state, ...
        'StateTransitionJacobianFcn', dfdx, ...
        'MeasurementJacobianFcn', @singermeasjac, ...
        'HasMeasurementWrapping', true, ...
        'StateCovariance', stateCov, ...
        'MeasurementNoise', measurementNoise, ...
        'HasAdditiveProcessNoise', true, ...
        'ProcessNoise', Q ...
        );
    setMeasurementSizes(EKF, n, n); % Helps coder deduce the measurement sizes
end