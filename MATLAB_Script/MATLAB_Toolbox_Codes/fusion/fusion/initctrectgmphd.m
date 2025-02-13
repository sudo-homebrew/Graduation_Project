function filter = initctrectgmphd(detections)
% INITCTRECTGMPHD Constant turn-rate rectangular target gmphd initialization
%   phd = INITCTRECTGMPHD initializes a constant turn-rate
%   rectangular target gmphd (Gaussian mixture probability hypothesis 
%   density) filter with zero components.
%
%   phd = INITCTRECTGMPHD(detections) initializes a constant turn-rate
%   rectangular target gmphd filter based on the information provided in
%   the input, detections. detections must be a cell array of
%   objectDetection objects. Each element of the cell array must specify
%   the measurement parameters, as a struct with the following fields.
%   Default values are used when a field is missing:
%     Frame           - 'rectangular' or 'spherical'. Default: 'rectangular'
%     OriginPosition  - a 3-by-1 real vector.  Default: [0;0;0]
%     OriginVelocity  - a 3-by-1 real vector.  Default: [0;0;0]
%     Orientation     - a 3-by-3 orthonormal
%                       orientation matrix.    Default: eye(3)
%     HasElevation    - a logical scalar.      Default: true, elevation is measured
%     HasVelocity     - a logical scalar.      Default: false in rectangular, true in spherical
%	  IsParentToChild - a logical scalar.      Default: false, orientation defines a rotation from child to parent frame
%
%   The function initializes a constant turn-rate rectangular target state
%   with the convention followed by ctrect and ctrectmeas:
%
%   [x;y;speed;theta;omega;length;width].
%
%   Notes:
%   ------
%   1. You can use INITCTRECTGMPHD as the FilterInitializationFcn property
%   for the trackingSensorConfiguration.
%   2. When detections are provided as input, the function adds 1 component 
%   to the mixture which reflects the mean of the detections.
%   3. The function defines the length and width of the rectangle using the
%   spread of measurements.
%   4. The function configures the process noise of the filter
%   by assuming a unit acceleration and yaw-acceleration standard deviation.
%   5. The function specifies a maximum of 500 components in the filter.
%   6. The function configures the covariance of the state using a unit 
%   covariance in observed dimensions (only length if measurement is from
%   one side only).
%
%   Example: Detections with position measurements in rectangular frame.
%   ----------------------------------------------------------------------
%   % Load detections generated by a rectangular target
%   load ('rectangularTargetDetections', 'detections', 'truthState');
%   
%   % Initialize the filter using detections
%   phd = initctrectgmphd(detections);
%
%   % Analyze the state of filter and truth
%   estState = phd.States;
%
%   disp(estState);
%   disp(truthState);
%
%   See also: ctrect, ctrectjac, ctrectmeas, ctrectmeasjac, 
%   trackingSensorConfiguration

% Copyright 2019 The MathWorks, Inc.

%#codegen

filter = gmphd(zeros(7,0),repmat(eye(7),[1 1 0]),...
    'StateTransitionFcn',@ctrect,...
    'StateTransitionJacobianFcn',@ctrectjac,...
    'HasAdditiveProcessNoise',false,...
    'ProcessNoise',eye(2),...
    'MeasurementFcn',@ctrectmeas,...
    'MeasurementJacobianFcn',@ctrectmeasjac,...
    'HasExtent', true,...
    'HasAdditiveMeasurementNoise',true,...
    'MaxNumComponents',500,...
    'MeasurementOrigin','extent');

if nargin == 0
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

[isRect,~,~,~,~,hasEl] = matlabshared.tracking.internal.fusion.parseDetectionForMeasurementParameters(detectionCells{1}, funcName, classToUse);

if isRect
    xDets = zAll(1,:);
    yDets = zAll(2,:);
else
    azDets = zAll(1,:);
    rngIndex = 2 + hasEl;
    rDets = zAll(rngIndex,:);
    [xDets, yDets] = pol2cart(deg2rad(azDets),rDets);
end

zCell = [xDets;yDets];
e = bsxfun(@minus, zCell, mean(zCell,2));
spread = e*e'/n;
dims = real(sqrt(max(eps(classToUse),eig(spread))));

length = 2*max(dims);
lengthCov = cast(1,classToUse);

if length/(2*min(dims)) < 5 % Width is observed
    theta = atan2d(2*min(dims),length);
    length = length*cosd(theta);
    width = length*sind(theta);
    widthCov = cast(1,classToUse);
else % Width is not observed
    % default width
    width = length/2;
    widthCov = cast(10, classToUse);
end

% Parse mean detection for position and velocity covariance.
[posMeas,velMeas,posCov,velCov] = matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(meanDetection,funcName,classToUse);

yaw = atan2d(velMeas(2),velMeas(1));

speed = norm(velMeas);

% Create a constant turn-rate rectangular state
state = zeros(7,1,classToUse);

state([1 2]) = posMeas(1:2); % Position 
state(3) = speed;
state(4) = yaw;
state(5) = 0;
state(6) = length;
state(7) = width;

% Transform velocity covariance to speed covariance and yaw covariance.
speedCov = [cosd(yaw)^2 sind(yaw)^2 2*cosd(yaw)*sind(yaw)]*[velCov(1,1);velCov(2,2);velCov(1,2)];
yawCov = 1/speed^2*[cosd(yaw)^2 sind(yaw)^2 -2*cosd(yaw)*sind(yaw)]*[velCov(1,1);velCov(2,2);velCov(1,2)];
yawCov(~isfinite(yawCov)) = 100; % unobservable yaw

% Constant covariance for omega
omegaCov = cast(100,classToUse);

% Assemble state covariance
stateCov = blkdiag(posCov(1,1),posCov(2,2),speedCov,yawCov,omegaCov,lengthCov,widthCov);

% Create filter
thisFilter = gmphd(state,stateCov);

% Append this state to the initialized filter
append(filter,thisFilter);
end