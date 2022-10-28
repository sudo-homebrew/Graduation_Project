function filter = zeroStateCVGSF(detection, numFilters, classToUse, hasMeasWrapping)
% This is an internal function and may be modified or removed in a future
% release.

% This function returns a 3-D constant velocity GSF with number of filters,
% numFilters. This functions helps to create a constant velocity filter
% with the right properties and size when the detection is invalid and
% state, measurement should not be set.

% Copyright 2019 The MathWorks, Inc.

%#codegen
state = zeros(6,1,classToUse);
stateCov = eye(6,classToUse);
processNoise = eye(3,classToUse);
stFcn = @constvel;
stjacFcn = @constveljac;
measFcn = @cvmeas;
measjacFcn = @cvmeasjac;

if nargin == 3
    hasMeasWrapping = false;
end

indFilters = cell(numFilters, 1);

for i = 1:numFilters
    indFilters{i} = trackingEKF(stFcn, measFcn, state, ...
        'StateCovariance', stateCov,...
        'ProcessNoise',processNoise,...
        'HasAdditiveProcessNoise', false,...
        'StateTransitionJacobianFcn',stjacFcn,...
        'MeasurementJacobianFcn',measjacFcn,...
        'HasAdditiveMeasurementNoise', true,...
        'HasMeasurementWrapping',hasMeasWrapping);
    n = numel(detection.Measurement);
    setMeasurementSizes(indFilters{i},n,n);
    indFilters{i}.MeasurementNoise = eye(n,classToUse);
end

filter = trackingGSF(indFilters, ones(1,numFilters, classToUse));