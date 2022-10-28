function [posParticles,velParticles,invalidDet] = sampleParticlesFromSphMeas(Detection, numParticles, origin, originVel, orientation, hasAz, hasEl, hasVel, hasRange, funcName)
%sampleParticleFromSphMeas   Sample particles from a detection in Spherical frame.
%  [posParticles,velParticles,invalidDet] =
%  sampleParticlesFromSphMeas(Detection,numParticles,origin,orientation,hasAz,hasEl,hasVel,hasRange,funcName)
%
%  Inputs:
%       Detection    - A scalar objectDetection 
%       numParticles - Number of samples to generate
%       origin       - origin position of the sensor.
%       originVel    - origin velocity of the sensor.
%       orientation  - sensor orientation as 3x3 matrix.
%       hasAz        - true, if detection has azimuth measurement.
%       hasEl        - true, if detection has elevation measurement.
%       hasVel       - true, if detection has range-rate.
%       hasRange     - true, if detection has range measurement.
%       funcName     - name of the function calling this internal function.
%
%   Outputs:
%       posParticle  - A numParticles-by-3 array of particle positions.
%       velParticles - A numParticles-by-3 array of particle velocities.
%       invalidDet   - true if detection is invalid.
%

% This is an internal function and may be removed in a future release

%   Copyright 2018 The MathWorks, Inc.

%#codegen

invalidDet = matlabshared.tracking.internal.fusion.isInvalidDetection(Detection);

if invalidDet
    posParticles = zeros(3,numParticles);
    velParticles = zeros(3,numParticles);
    return;
end
% Check state observability
fullStateObservable = hasAz && hasEl && hasRange;

% Create holders for normal and uniform distribution. Their sizes can be
% varied by using the reset method.
normalDist = matlabshared.tracking.internal.NormalDistribution(numel(Detection.Measurement));
uniformDist = matlabshared.tracking.internal.UniformDistribution(1);

% Validate the size of the measurement is as expected
expMeasSize = matlabshared.tracking.internal.fusion.getExpectedMeasurementSize(false,hasAz,hasEl,hasVel,hasRange);
validateattributes(Detection.Measurement, {'double','single'}, ...
    {'real','finite','vector','numel', expMeasSize}, funcName, 'Detection.Measurement');

if fullStateObservable % [az,el,r (rr)]
    normalDist.Mean = reshape(Detection.Measurement,1,[]); % Requires row vector
    normalDist.Covariance = Detection.MeasurementNoise;
    rAzElParticles = normalDist.sample(numParticles);
    rParticles = rAzElParticles(:,3);
    azParticles = rAzElParticles(:,1);
    elParticles = rAzElParticles(:,2);
    if hasVel
        rDotParticles = rAzElParticles(:,4);
    else
        rDotParticles = 10*randn(numParticles,1);
    end
else
    if isstruct(Detection.MeasurementParameters)
        measParam = Detection.MeasurementParameters;
    elseif iscell(Detection.MeasurementParameters)
        measParam = Detection.MeasurementParameters{1};
    else
        measParam = struct;
    end
    % For unobservable detection, check for parameters FieldOfView and
    % RangeLimits for uniform distribution on their dimensions.
    if isfield(measParam,'FieldOfView')
        fov = measParam.FieldOfView;
    else
        fov = [360 180]; % default: full view
    end
    
    if isfield(measParam,'RangeLimits')
        rangeLims = measParam.RangeLimits;
    else
        rangeLims = [1e3 2e3]; % default: [1-2 km]
    end
    
    % Divide measurement into different types for normal distribution on 
    % known and unknown variables

    if hasRange && ~hasAz && ~hasEl %[r (rr)]
        normalDist.Mean = reshape(Detection.Measurement,1,[]);
        normalDist.Covariance = Detection.MeasurementNoise;
        rParticles = normalDist.sample(numParticles);
        if hasVel
            rDotParticles = rParticles(:,2);
        else
            rDotParticles = 10*randn(numParticles,1);
        end
        azLimits = fov(1)/2; 
        elLimits = fov(2)/2;
        uniformDist.reset(2); 
        uniformDist.RandomVariableLimits = [-azLimits azLimits;-elLimits elLimits];
        azElParticles = uniformDist.sample(numParticles);
        azParticles = azElParticles(:,1);
        elParticles = azElParticles(:,2);
        
    elseif hasRange && (hasAz || hasEl) %[az/el r (rr)] available
        uniformDist.reset(1); % For elevation or azimuth
        azLimits = fov(1)/2;
        elLimits = fov(2)/2;
        normalDist.Covariance = Detection.MeasurementNoise;
        normalDist.Mean = reshape(Detection.Measurement,1,[]);
        rAngleParticles = normalDist.sample(numParticles);
        rParticles = rAngleParticles(:,2);
        if hasAz
            uniformDist.RandomVariableLimits = [-elLimits elLimits];
            pParticles = uniformDist.sample(numParticles);
            elParticles = pParticles(1:numParticles,1);
            azParticles = rAngleParticles(:,1);
        else
            uniformDist.RandomVariableLimits = [-azLimits azLimits];
            pParticles = uniformDist.sample(numParticles);
            azParticles = pParticles(1:numParticles,1);
            elParticles = rAngleParticles(:,1);
        end
        if hasVel
            rDotParticles = rAngleParticles(:,3);
        else
            rDotParticles = 10*randn(numParticles,1);
        end
        
    elseif  ~hasRange && hasEl && hasAz % [az el]
        uniformDist = matlabshared.tracking.internal.UniformDistribution(1); % For range
        uniformDist.RandomVariableLimits = rangeLims;
        pParticles = uniformDist.sample(numParticles);
        rParticles = pParticles(1:numParticles,1);
        normalDist.Covariance = Detection.MeasurementNoise;
        normalDist.Mean = reshape(Detection.Measurement,1,[]);
        azElParticles = normalDist.sample(numParticles);
        azParticles = azElParticles(:,1);
        elParticles = azElParticles(:,2);
        rDotParticles = 10*randn(numParticles,1);
    else % just angle [az or el]
        normalDist.Mean = reshape(Detection.Measurement,1,[]); % just angle
        normalDist.Covariance = Detection.MeasurementNoise;
        pParticles = normalDist.sample(numParticles);
        uniformDist = matlabshared.tracking.internal.UniformDistribution(2); % For [unAvailableangle,r]
        azLimits = fov(1)/2;
        elLimits = fov(2)/2;
        if hasAz
            uniformDist.RandomVariableLimits = [-elLimits/2 elLimits/2;rangeLims];
            angleRParticles = uniformDist.sample(numParticles);
            elParticles = angleRParticles(1:numParticles,1);
            azParticles = pParticles(1:numParticles,1);
        else
            uniformDist.RandomVariableLimits = [-azLimits/2 azLimits/2;rangeLims];
            angleRParticles = uniformDist.sample(numParticles);
            azParticles = angleRParticles(1:numParticles,1);
            elParticles = pParticles(1:numParticles,1);
        end
        rParticles = angleRParticles(1:numParticles,2);
        rDotParticles = 10*randn(numParticles,1);
    end
end

tgtPosParticles = [azParticles elParticles rParticles]';
posParticles = matlabshared.tracking.internal.fusion.local2globalcoord(tgtPosParticles,'sr',origin,orientation);

% Sample velocity particles
crossVelY = 10*randn(numParticles,1); %Large number similar to other tracking filters
crossVelZ = 10*randn(numParticles,1); %Large number similar to other tracking filters
% Transform velocity to sensor's cartesian coordinates
cosAz = cosd(azParticles);
sinAz = sind(azParticles);
cosEl = cosd(elParticles);
sinEl = sind(elParticles);
xVel = rDotParticles.*cosEl.*cosAz - crossVelZ.*sinEl.*cosAz - crossVelY.*cosEl.*sinAz;
yVel = rDotParticles.*cosEl.*sinAz - crossVelZ.*sinEl.*sinAz + crossVelY.*cosEl.*cosAz;
zVel = rDotParticles.*sinEl + crossVelZ.*cosEl;
tgtVelParticles = [xVel yVel zVel]';
velParticles = matlabshared.tracking.internal.fusion.local2globalcoord(tgtVelParticles,'rr',originVel,orientation);

end



