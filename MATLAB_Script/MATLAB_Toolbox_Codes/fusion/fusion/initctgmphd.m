function phd = initctgmphd(detections)
% INITCTGMPHD Constant turn-rate gmphd initialization
%   phd = INITCTGMPHD initializes a constant turn-rate gmphd
%   (Gaussian-mixture probability hypothesis density) filter with zero
%   components.
%
%   phd = INITCTGMPHD(detections) initializes a constant turn-rate gmphd
%   filter based on the information provided in the input, detections.
%   detections must be a cell array of objectDetection objects. Each
%   element of the cell array must specify the measurement parameters, as a
%   struct with the following fields. Default values are used when a field
%   is missing:
%     Frame	          - 'rectangular' or 'spherical'. Default: 'rectangular'
%     OriginPosition  - a 3-by-1 real vector.  Default: [0;0;0]
%     OriginVelocity  - a 3-by-1 real vector.  Default: [0;0;0]
%     Orientation     - a 3-by-3 orthonormal
%                       orientation matrix.    Default: eye(3)
%     HasElevation    - a logical scalar.      Default: true, elevation is measured
%     HasVelocity     - a logical scalar.      Default: false in rectangular, true in spherical
%	  IsParentToChild - a logical scalar.      Default: false, orientation defines a rotation from child to parent frame
%
%   The function initializes a constant turn rate state with the
%   convention followed by constturn and ctmeas, [x;vx;y;vy;w;z;vz].
%
%   Notes:
%   ------
%   1.  You can use INITCTGMPHD as the FilterInitializationFcn property
%       for the trackingSensorConfiguration.
%   2.  When detections are provided as input, the function adds 1
%       component to the density which reflects the mean of the detections.
%   3.  The function uses the spread of measurements to specify the
%       positional covariance.
%   4.  The function configures the process noise of the filter
%       by assuming a unit acceleration standard deviation and a unit
%       angular acceleration standard deviation.
%   5.  The function specifies a maximum of 500 components in the filter.
%   6.  The function specifies that the filter has extended objects if
%       number of input detections are greater than 1.
%
%   Example 1: Detection with position measurement from a point object
%   ------------------------------------------------------------------
%   % The constant turn-rate measurement function, ctmeas, provides a
%   % position measurement in 3-D. For example: [1;2;3] where x = 1, y = 2
%   % and z = 3 refers to x,y and z position of the object. Consider a
%   % point object located at [1;2;3]
%   detection = objectDetection(0,[1;2;3]);
%   phd = INITCTGMPHD(detection);
%   
%   % Check that the values of State has the same position estimate as the
%   % measurement
%   phd.States
%   
%   % Check that the filter is specified to track point objects
%   phd.HasExtent
%
%   Example 2: Detections with position measurements from an extended object
%   ------------------------------------------------------------------------
%   % Consider an object located at position [1;2;3] with detections 
%   % uniformly spread around it's extent of size 1.2, 2.3 and 3.5 in x, y 
%   % and z direction respectively.
%   detections = cell(20,1);
%   location = [1;2;3];
%   dimensions = [1.2;2.3;3.5];
%   measurements = location + dimensions.*(-1 + 2*rand(3,20));
%   for i = 1:20
%       detections{i} = objectDetection(0,measurements(:,i));
%   end
%   phd = INITCTGMPHD(detections);
%   % Check the values of State has the same position
%   % estimates as the mean of measurements.
%   phd.States
%   mean(measurements,2)
%   
%   % Check that the filter is specified to track extended objects
%   phd.HasExtent
%
%   See also: constturn, constturnjac, ctmeas, ctmeasjac, initcvggiwphd,
%   initcaggiwphd, trackingSensorConfiguration
%

%   Copyright 2018-2019 The MathWorks, Inc.

%#codegen

if nargin == 0
    classToUse = 'double';
    phd = gmphd(zeros(7,0),repmat(eye(7),[1 1 0]),...
                    'MaxNumComponents',2000,...
                    'ProcessNoise',eye(4,classToUse),...
                    'HasAdditiveProcessNoise',false);
    phd.MeasurementFcn = @ctmeas;
    phd.MeasurementJacobianFcn = @ctmeasjac;
    phd.HasAdditiveMeasurementNoise = true;
    phd.StateTransitionFcn = @constturn;
    phd.StateTransitionJacobianFcn = @constturnjac;
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

% Use spread to specify noise
e = bsxfun(@minus, zAll, z);
meanDetection.MeasurementNoise = R + (e*e')/n;

[posMeas,velMeas,posCov,velCov] = matlabshared.tracking.internal.fusion.parseDetectionForInitFcn(meanDetection,funcName,classToUse);
states = zeros(7,1,classToUse);
covariances = zeros(7,7,classToUse);
posIndex = [1 3 6];
velIndex = [2 4 7];
states(posIndex) = posMeas;
states(velIndex) = velMeas;

% Set the covariances
covariances(posIndex,posIndex) = posCov;
covariances(velIndex,velIndex) = velCov;
wCov = cast(100,classToUse);
covariances(5,5) = wCov;

phd = gmphd(states,covariances,...
    'MaxNumComponents',500,...
    'ProcessNoise',eye(4,classToUse),...
    'HasAdditiveProcessNoise',false);

phd.MeasurementFcn = @ctmeas;
phd.MeasurementJacobianFcn = @ctmeasjac;
phd.HasAdditiveMeasurementNoise = true;
phd.StateTransitionFcn = @constturn;
phd.StateTransitionJacobianFcn = @constturnjac;
phd.HasExtent = n > 1;

end