function [fusedPos] = triangulateAngularState(state,stateCovariance,sensorPositions,sensorOrientations)
% This is an internal function and may be removed or modified in a future 
% release.
% triangulateAngularState Triangulate multiple angle and their rate states
% to form positions in 3-D space.
% This function computes the initial state of a target using angle measurements
% from multiple sensors. It uses the following sub-optimal Linear Least Squares problem
% Ax = b
% A:
% [cos(El1)*cos(Az1) -cos(El2)*cos(Az2)         0           
%            0        cos(El2)*cos(Az2) -cos(El3)*cos(Az3)
%  cos(El1)*sin(Az1) -cos(El2)*sin(Az2)         0
%            0        cos(El2)*sin(Az2) -cos(El3)*sin(Az3)
%         sin(El1)         -sin(El2)            0
%            0              sin(El2)        -sin(El3)     ]
% x: 
% [r1 
%  r2 
%  r3] : ri: Range of Target from ith sensor
% b:
% [xSensor1 - xSensor2
%  xSensor2 - xSensor3
%  .
%  .
%  ySensor1 - ySensor2
%  ySensor2 - ySensor3
%  zSensor1 - zSensor2
%  zSensor2 - zSensor3]
% The output argument cost defines the -log(likelihood) of fusing these 
% measurements.
%

% Copyright 2018 The MathWorks, Inc.

%#codegen

hasVelocity = size(state,2) > 2;

if hasVelocity
    azIndex = 1;
    elIndex = 3;
    azDotIndex = 2;
    elDotIndex = 4;
else
    azIndex = 1;
    elIndex = 2;
    azDotIndex = -1;
    elDotIndex = -1;
end

numSensors = size(sensorPositions,1);

% Concatenate azimuth and elevation measurements
az = deg2rad(state(:,azIndex));
el = deg2rad(state(:,elIndex));
azSigma = deg2rad(sqrt(stateCovariance(azIndex,azIndex,:))).^2;
elSigma = deg2rad(sqrt(stateCovariance(elIndex,elIndex,:))).^2;

if hasVelocity
    azDot = deg2rad(state(:,azDotIndex));
    elDot = deg2rad(state(:,elDotIndex));
    azDotSigma = deg2rad(sqrt(stateCovariance(azDotIndex,azDotIndex,:))).^2;
    elDotSigma = deg2rad(sqrt(stateCovariance(elDotIndex,elDotIndex,:))).^2;
    azAzDotSigma = deg2rad(sqrt(stateCovariance(azIndex,azDotIndex,:))).^2;
    elElDotSigma = deg2rad(sqrt(stateCovariance(elIndex,elDotIndex,:))).^2;
end

% Rotation from spherical sensor to cartesian sensor frame
xFactor = cos(el).*cos(az); %[u1sx;u2sx;u3sx ....];
yFactor = cos(el).*sin(az); % [u1sy;u2sy;u3sy ...];
zFactor = sin(el); %[u1sz;u2sz;u3sz....];
sphFactor = zeros(numSensors*3,1); % Store [u1sx;u1sy;u1sz;u2sx;u2sy;u2sz...];
sphFactor(1:3:end) = xFactor;
sphFactor(2:3:end) = yFactor;
sphFactor(3:3:end) = zFactor;

rotTimesFactor = sensorOrientations*sphFactor; % [u1x;u1y;u1z;u2x;u2y;u2z]

diffSensorPos = diff(sensorPositions,1,1); %[s1x - s2x, s1y - s2y, s1z - s2z]
rotXFactor = rotTimesFactor(1:3:end); % [u1x;u2x;u3x;u4x];
rotYFactor = rotTimesFactor(2:3:end); % [u1y;u2y;u3y;u4y];
rotZFactor = rotTimesFactor(3:3:end); % [u1z;u2z;u3z;u4z];

% Construct Least Squares Problem
% A = [Ax;Ay;Az]
% b = [s1x-s2x;s2x-s3x;...;s1y-s2y;s2y-s3y;.....;s1z - s2z;s2z-s3z];

% Ax = [u1x -u2x 0 0 0 ...; 0 u2x -u3x 0 0 0 ...;...]
Ax = zeros(numSensors-1,numSensors);  
Ax(1:numSensors:end) = rotXFactor(1:end-1); 
Ax(numSensors:numSensors:end) = -rotXFactor(2:end);

% Ay = [u1y -u2y 0 0 0 ...; 0 u2y -u3y 0 0 0 ...;...]
Ay = zeros(numSensors-1,numSensors);  
Ay(1:numSensors:end) = rotYFactor(1:end-1);
Ay(numSensors:numSensors:end) = -rotYFactor(2:end);

% Az = [u1z -u2z 0 0 0 ...; 0 u2z -u3z 0 0 0 ...;...]
Az = zeros(numSensors-1,numSensors);  
Az(1:numSensors:end) = rotZFactor(1:end-1);
Az(numSensors:numSensors:end) = -rotZFactor(2:end);

% Solve Least Squares to get range estimate for each sensor
A = [Ax;Ay;Az];
b = diffSensorPos(:);

% Check if the system is observable, otherwise return range estimates as 0.
if rank(A) >= numSensors
    rangeEstimates = A\b;
else
    rangeEstimates = zeros(numSensors,1,class(A));
end
% negative range estimates mean diverging angle-only measurements, set
% range as 0 to make sure the estimate from this sensor is reported at 
% it's own position.
rangeEstimates(rangeEstimates < 0) = 0;

% Transform rangeEstimates into global positions
uVecs = reshape(rotTimesFactor,3,[]);
globalPos = bsxfun(@times,uVecs,rangeEstimates');
px = globalPos(1,:)';
py = globalPos(2,:)';
pz = globalPos(3,:)';

pos = zeros(3,1);
pos(1) = mean(px + sensorPositions(:,1));
pos(2) = mean(py + sensorPositions(:,2));
pos(3) = mean(pz + sensorPositions(:,3));

% Compute rangeDotEstimates. The equations are as follows:
% Ax = b
% ADot*x + A*xDot = bDot
% A*xDot = bDot - ADot*x;
% x = rangeEstimates;
% xDot = A\bDot;

if hasVelocity
    xDotFactor = -cos(el).*sin(az).*azDot - sin(el).*cos(az).*elDot;
    yDotFactor = cos(el).*cos(az).*azDot - sin(el).*sin(az).*elDot;
    zDotFactor = cos(el).*elDot;
    sphVelFactor = zeros(numSensors*3,1);
    sphVelFactor(1:3:end) = xDotFactor;
    sphVelFactor(2:3:end) = yDotFactor;
    sphVelFactor(3:3:end) = zDotFactor;
    rotTimesDotFactor = sensorOrientations*sphVelFactor;
    rotXDotFactor = rotTimesDotFactor(1:3:end);
    rotYDotFactor = rotTimesDotFactor(2:3:end);
    rotZDotFactor = rotTimesDotFactor(3:3:end);
    AxDot = zeros(numSensors-1,numSensors);
    AyDot = zeros(numSensors-1,numSensors);
    AzDot = zeros(numSensors-1,numSensors);
    AxDot(1:numSensors:end) = rotXDotFactor(1:end-1);
    AxDot(numSensors:numSensors:end) = -rotXDotFactor(2:end);
    AyDot(1:numSensors:end) = rotYDotFactor(1:end-1);
    AyDot(numSensors:numSensors:end) = -rotYDotFactor(2:end);
    AzDot(1:numSensors:end) = rotZDotFactor(1:end-1);
    AzDot(numSensors:numSensors:end) = -rotZDotFactor(2:end);
    ADot = [AxDot;AyDot;AzDot];
    sensorVelocities = cat(1,measurementParameters.OriginVelocity);
    diffSensorVel = diff(sensorVelocities,1,1);
    bDot = diffSensorVel(:) - ADot*rangeEstimates;
    rangeDotEstimates = A\bDot;
    vx = rangeDotEstimates.*cos(el).*cos(az) - rangeEstimates.*sin(el).*cos(az).*elDot - rangeEstimates.*cos(el).*sin(az).*azDot;
    vy = rangeDotEstimates.*cos(el).*sin(az) - rangeEstimates.*sin(el).*sin(az).*elDot + rangeEstimates.*cos(el).*cos(az).*azDot;
    vz = rangeDotEstimates.*sin(el) + rangeEstimates.*cos(el).*elDot;
    vel = [mean(vx);mean(vy);mean(vz)];
else
    vel = zeros(3,1,'like',state);
end
initGuess = zeros(6,1,'like',state);
initGuess(1:2:end) = pos;
initGuess(2:2:end) = vel;
fusedPos = initGuess(1:2:end);


