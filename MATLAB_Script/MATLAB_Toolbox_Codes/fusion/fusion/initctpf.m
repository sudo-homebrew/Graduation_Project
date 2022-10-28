function PF = initctpf(Detection)
%INITCTPF  Constant turn rate trackingPF initialization
%   PF = INITCTPF(Detection)   initializes a constant turn rate (CT)
%   trackingPF (Particle Filter) based on information provided in Detection.
%   Detection must be an objectDetection. The detection must specify the measurement
%   parameters, as a struct with the following fields. Default values are
%   used when a field is missing:
%     Frame           - 'rectangular' or 'spherical'. Default: 'rectangular'
%     OriginPosition  - a 3-by-1 real vector.  Default: [0;0;0]
%     OriginVelocity  - a 3-by-1 real vector.  Default: [0;0;0]
%     Orientation     - a 3-by-3 orthonormal 
%                       orientation matrix.    Default: eye(3)
%     HasRange        - a logical scalar       Default: true, range is measured
%     HasAzimuth      - a logical scalar       Default: true, azimuth is measured
%     HasElevation    - a logical scalar.      Default: true, elevation is measured
%     HasVelocity     - a logical scalar.      Default: false in rectangular, true in spherical if range is measured
%     IsParentToChild - a logical scalar.      Default: false, orientation defines a rotation from child to parent frame
%
%   Notes:
%   ------
%   1. You can use INITCTPF as the FilterInitializationFcn property.
%   2. The function configures the filter with 1000 particles.
%   3. In creating the filter the function configures the process noise
%      assuming a unit acceleration standard deviation and a unit angular
%      acceleration standard deviation.
%
%   Example 1 - detection with position measurement in rectangular frame
%   --------------------------------------------------------------------
%   % The constant turn measurement function, ctmeas, provides a
%   % position measurement in 3-D. For example: x=1, y=3, z=0.
%   % Use a 3-D position measurement noise [1 0.2 0; 0.2 2 0; 0 0 1];
%   detection = objectDetection(0, [1;3;0], 'MeasurementNoise', [1 0.2 0; 0.2 2 0; 0 0 1]);
% 
%   % Use INITCTPF to create a trackingPF filter initialized at the
%   % provided position and using the measurement noise defined above.
%   pf1 = initctpf(detection);
%
%   % Check the values of the state and measurement noise.
%   % Verify that the filter state, pf1.State, has the approximately the same
%   % position components as detection measurement, detection.Measurement.
%   % Verify that the filter measurement noise, pf1.MeasurementNoise, is the same
%   % as the detection.MeasurementNoise values.
%   pf1.State
%   pf1.MeasurementNoise
%
%   Example 2 - detection with position measurement in spherical frame
%   ------------------------------------------------------------------
%   % The constant turn measurement function, ctmeas, provides a
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
%   % Use INITCTPF to create a trackingPF filter initialized at the 
%   % provided position and using the measurement noise defined above.
%   pf2 = initctpf(detection);
%
%   % Check that the returned measurement from the filter is approximately
%   % same as the detection.Measurement.
%   ctmeas(pf2.State,detection.MeasurementParameters)
%
%   See also: trackingPF, objectDetection, initctekf, initctukf,
%   initctckf, initcvpf, initcapf, constturn, ctmeas

%#codegen

% Copyright 2018 The MathWorks, Inc.

% Input validation:
funcName = mfilename;
matlabshared.tracking.internal.fusion.validateFilterInitializationInput(Detection, funcName);

classToUse = class(Detection.Measurement);


%% How to Initialize a Tracking Particle Filter?
% 1. For a rectangular frame, initialize the particle filter using state
% and state covariance from measurements in rectangular frame.
% 2. For a spherical frame, check if the positional state is observable 
% from measurement
% 3. If the state is observable, use measurement and measurement noise to
% generate normally distributed particles in spherical frame.
% 4. If the state is unobservable, use uniform distribution along
% unobservable states 
% 5. Transform the spherical particles into rectangular frame using
% non-linear transformation.

%% Parse measurement parameters to decide method used for initialization
[isRect, origin, originVel, orientation, hasAz, hasEl, hasVel, hasRange] = ...
    matlabshared.tracking.internal.fusion.parseDetectionForMeasurementParameters(Detection,funcName,classToUse);

% Create Process Noise matrix
scaleAccel = ones(1,classToUse);
scaleAngularAccel = ones(1,classToUse);
Q = blkdiag(scaleAccel^2, scaleAccel^2, scaleAngularAccel^2, scaleAccel^2);


% Store measurement properties
n = numel(Detection.Measurement);
if isscalar(Detection.MeasurementNoise)
    measurementNoise = cast(matlabshared.tracking.internal.expandScalarValue(Detection.MeasurementNoise, [n, n]),classToUse);
else
    measurementNoise = cast(Detection.MeasurementNoise,classToUse);
end

% Number of particles
numParticles = 1000;


%% Initialize the Particle Filter in Rectangular frame using state, state covariance
if isRect
    [posMeas, velMeas, posCov, velCov, invalidDet] = ...
        matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(Detection, funcName, classToUse);
    Hpos = cast([1 0 0 0 0 0 0; 0 0 1 0 0 0 0; 0 0 0 0 0 1 0],classToUse); % position = Hpos*state
    Hvel = cast([0 1 0 0 0 0 0; 0 0 0 1 0 0 0; 0 0 0 0 0 0 1],classToUse); % velocity = Hvel*state
    state = Hpos' * posMeas(:) + Hvel'*velMeas(:);
    L = 100; % A large value compared to the expected measurement noise.
    omegaCov = cast(diag([0,0,0,0,L,0,0]),classToUse);
    stateCov = Hpos'*posCov*Hpos + Hvel'*velCov*Hvel + omegaCov;
    if ~invalidDet
        PF = trackingPF(@constturn,@ctmeas,state,'NumParticles',numParticles,...
            'StateCovariance',stateCov,'HasAdditiveProcessNoise', false, ...
            'ProcessNoise',Q,'MeasurementNoise',measurementNoise);
        setMeasurementSizes(PF,n,n);
    else
        PF = trackingPF(@constturn,@ctmeas,state,'NumParticles',numParticles,...
            'StateCovariance',stateCov,'HasAdditiveProcessNoise', false, ...
            'ProcessNoise',Q);
    end
else
%% Initialize the Particle Filter in Spherical Frame using sample generation 
[posParticles,velParticles,invalidDet] = fusion.internal.sampleParticlesFromSphMeas(Detection, numParticles, origin, originVel, orientation, hasAz, hasEl, hasVel, hasRange, funcName);

if ~invalidDet
    PF = trackingPF(@constturn,@ctmeas,zeros(7,1),'NumParticles',numParticles,...
        'HasAdditiveProcessNoise', false,'ProcessNoise',Q,...
        'MeasurementNoise',measurementNoise);
    setMeasurementSizes(PF,n,n);
else
    PF = trackingPF(@constturn,@ctmeas,zeros(7,1),'NumParticles',numParticles,...
        'HasAdditiveProcessNoise', false,'ProcessNoise',Q);
end

% Modify the Particles to generated position and velocity particles.
PF.Particles(1,:) = posParticles(1,:);
PF.Particles(3,:) = posParticles(2,:);
PF.Particles(6,:) = posParticles(3,:); 
PF.Particles(2,:) = velParticles(1,:);
PF.Particles(4,:) = velParticles(2,:);
PF.Particles(7,:) = velParticles(3,:);

end
