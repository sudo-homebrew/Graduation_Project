function phd = initcvggiwphd(detections)
% INITCVGGIWPHD Constant velocity ggiwphd initialization
%   phd = INITCVGGIWPHD initializes a constant velocity ggiwphd (Gamma
%   Gaussian Inverse Wishart Probability Hypothesis Density) filter with
%   zero components.
%
%   phd = INITCVGGIWPHD(detections) initializes a constant velocity ggiwphd
%   filter based on the information provided in the input, detections.
%   detections must be a cell array of objectDetection objects. Each
%   element of the cell array must specify the measurement parameters, as a
%   struct with the following fields. Default values are used when a field
%   is missing:
%     Frame			  - 'rectangular' or 'spherical'. Default: 'rectangular'
%     OriginPosition  - a 3-by-1 real vector.  Default: [0;0;0]
%     OriginVelocity  - a 3-by-1 real vector.  Default: [0;0;0]
%     Orientation     - a 3-by-3 orthonormal
%                       orientation matrix.    Default: eye(3)
%     HasElevation    - a logical scalar.      Default: true, elevation is measured
%     HasVelocity     - a logical scalar.	   Default: false in rectangular, true in spherical
%	  IsParentToChild - a logical scalar.	   Default: false, orientation defines a rotation from child to parent frame
%
%   The function initializes a constant velocity state with the convention 
%   followed by constvel and cvmeas, [x;vx;y;vy;z;vz].
%
%   Notes:
%   ------
%   1. You can use INITCVGGIWPHD as the FilterInitializationFcn property
%   for the trackingSensorConfiguration.
%   2. When detections are provided as input, the function adds 1 component 
%   to the density which reflects the mean of the detections. 
%   3. The function uses the spread of measurements to describe the
%   Inverse-Wishart distribution.
%   4. The function uses the number of detections to describe the Gamma
%   distribution.
%   5. The function configures the process noise of the filter
%   by assuming a unit acceleration standard deviation.
%   6. The function specifies a maximum of 500 components in the filter.
%
%   Example: Detections with position measurements in rectangular frame.
%   ----------------------------------------------------------------------
%   % The constant velocity measurement function, cvmeas, provides a
%   % position measurement in 3-D. For example: [1;2;3] where x = 1, y = 2
%   % and z = 3 refers to x,y and z position of the object. Consider an
%   % object located at position [1;2;3] with detections uniformly spread
%   % around it's extent of size 1.2, 2.3 and 3.5 in x, y and z direction
%   % respectively.
%   detections = cell(20,1);
%   location = [1;2;3];
%   dimensions = [1.2;2.3;3.5];
%   measurements = location + dimensions.*(-1 + 2*rand(3,20));
%   for i = 1:20
%       detections{i} = objectDetection(0,measurements(:,i));
%   end
%   phd = INITCVGGIWPHD(detections);
%   % Check the values of State has the same position
%   % estimates as the mean of measurements.
%   phd.States
%   mean(measurements,2)
%   
%   % Check the values of Extent and Expected number of detections
%   extent = phd.ScaleMatrices/(phd.DegreesOfFreedom - 4)
%   expDetections = phd.Shapes/phd.Rates
%
%   See also: constvel, constveljac, cvmeas, cvmeasjac, initctggiwphd,
%   initcaggiwphd, trackingSensorConfiguration
%

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

if nargin == 0
    classToUse = 'double';
    phd = ggiwphd(zeros(6,0,classToUse),repmat(eye(6,classToUse),[1 1 0]),...
        'ScaleMatrices',zeros(3,3,0,classToUse),...
        'MaxNumComponents',500,...
        'ProcessNoise',eye(3,classToUse),...
        'HasAdditiveProcessNoise',false);
    phd.MeasurementFcn = @cvmeas;
    phd.MeasurementJacobianFcn = @cvmeasjac;
    phd.PositionIndex = [1 3 5];
    phd.ExtentRotationFcn = @(x,dT)eye(3,class(x));
    phd.HasAdditiveMeasurementNoise = true;
    phd.StateTransitionFcn = @constvel;
    phd.StateTransitionJacobianFcn = @constveljac;
    return;
end
funcName = mfilename;

% Validate detection input
validateattributes(detections,{'objectDetection','cell'},{'vector'},funcName,'Detections',1);
if isa(detections,'objectDetection')
    detectionCells = matlabshared.tracking.internal.fusion.makeDetectionCells(detections);
else
    validateattributes(detections{1},{'objectDetection'},{'scalar'},funcName,'detections{:}',1);
    detectionCells = detections;
end
% The mixture is initialized with one component whose position and velocity
% is governed by the mean of all measurements.

% Allocate memory for meanDetection.
meanDetection = detectionCells{1};
classToUse = class(meanDetection.Measurement);

n = numel(detectionCells);

% Collect all measurements and measurement noises.
if coder.target('MATLAB')
    allDets = [detectionCells{:}];
    zAll = horzcat(allDets.Measurement);
    RAll = cat(3,allDets.MeasurementNoise);
else
    p = numel(detectionCells{1}.Measurement);
    zAll = zeros(p,n,classToUse);
    RAll = zeros(p,p,n,classToUse);
    for i = 1:n
        zAll(:,i) = detectionCells{i}.Measurement;
        RAll(:,:,i) = detectionCells{i}.MeasurementNoise;
    end
end

% Specify mean noise and measurement
z = mean(zAll,2);
R = mean(RAll,3);
meanDetection.Measurement = z;
meanDetection.MeasurementNoise = R;

% Parse mean detection for position and velocity covariance.
[posMeas,velMeas,posCov,velCov] = matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(meanDetection,funcName,classToUse);

% Create a constant velocity state and covariance
states = zeros(6,1,classToUse);
covariances = zeros(6,6,classToUse);
states(1:2:end) = posMeas;
states(2:2:end) = velMeas;
covariances(1:2:end,1:2:end) = posCov;
covariances(2:2:end,2:2:end) = velCov;

% The extent is set to the spread of the measurements in positional-space.
e = bsxfun(@minus,zAll,z);
Z = e*e'/n + R;
v = cast(100 + n,classToUse);
% Meas to state jacobian
p = detectionCells{1}.MeasurementParameters;
if iscell(p)
    H = cvmeasjac(states,p{:});
else
    H = cvmeasjac(states,p);
end
Bk = H(:,1:2:end);
Bk2 = eye(3)/Bk;
V = (v-4)*Bk2*Z*Bk2';

% Measurement rate is set using number of measurements
alpha = cast(n, classToUse);
beta = cast(1, classToUse);

phd = ggiwphd(states,covariances,...
    'DegreesOfFreedom',v,...
    'ScaleMatrices',V,...
    'Shapes',alpha,'Rates',beta,...
    'MaxNumComponents',500,...
    'ProcessNoise',eye(3,classToUse),...
    'HasAdditiveProcessNoise',false);
phd.TemporalDecay = cast(100,classToUse);
phd.GammaForgettingFactors = cast(1.03,classToUse);
phd.MeasurementFcn = @cvmeas;
phd.MeasurementJacobianFcn = @cvmeasjac;
phd.PositionIndex = [1 3 5];
phd.ExtentRotationFcn = @(x,dT)eye(3,class(x));
phd.HasAdditiveMeasurementNoise = true;
phd.StateTransitionFcn = @constvel;
phd.StateTransitionJacobianFcn = @constveljac;

end