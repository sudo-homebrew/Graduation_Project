function [partitions, varargout] = partitionDP(detections, varargin)
% This is an internal function and may be removed or modified in a future
% release.
%
% partitions = fusion.internal.partitionDP(detections) produces partitions
% using Distance-partitioning algorithm. The lower and upper thresholds are
% 0.5 and 6.25. The function uses Mahalanobis distance between detections
% to auto-compute partitioning thresholds.
%
% partitions = fusion.internal.partitionDP(detections,tLower,tUpper) allows
% you to specify lower and upper thresholds. Specify the 'MaxNumPartitions'
% Name-value pair to control the maximum number of partitions generated.
%
% [partitions, idx] = fusion.internal.partitionDP(detections, thresholds)
% allows you to specify exact thresholds for partitioning. idx provides a
% mapping between specified thresholds and generated partitions.
%
% partitions = fusion.internal.partitionDP(...,'Distance',distanceType)
% allows you to control the distance metric between detections. By default,
% distanceType is 'Mahalanobis'. Other choices are 'Euclidean'.
%
% partitions = fusion.internal.partitionDP(detections, tLower, tUpper,'MaxNumPartitions',maxNum)
% allows to control the maximum number of partitions when thresholds are
% computed automatically.

% Copyright 2021 The MathWorks, Inc.

%#codegen

% Parse inputs
[calculateThreshold, threshold, maxNumPartitions, distanceType, dataType] = fusion.internal.parsePartitionDPInputs(detections, varargin{:});

% Unique sensors in the list
sensorIdx = fusion.internal.concatenateDetectionData(detections,'SensorIndex');
uqSensors = unique(sensorIdx);

% auto-compute partitioning thresholds
if calculateThreshold
    % Calculate inter detection distance per sensor and use the entire
    % distance matrix for auto-computation of partition thresholds
    distanceMatrix = zeros(numel(detections),dataType);
    for i = 1:numel(uqSensors)
        thisSensor = sensorIdx == uqSensors(i);
        thisSensorDetections = fusion.internal.selectDetections(detections,thisSensor);
        distanceMatrix(thisSensor,thisSensor) = fusion.internal.interDetectionDistance(thisSensorDetections, distanceType);
    end
    
    % For auto-computation thresholds are computed only using second
    % decimal accuracy to prevent different partitions created for close
    % thresholds , example: threshold = 1.001 and threshold = 1.002. floor
    % will keep the threshold below distance levels, making sure a
    % threshold exists between each distance level separated by minimum
    % 0.01 distance.
    dBetweenLevels = floor(distanceMatrix(:)*100)/100;
    
    % Add large threshold to cover the level between highest distance and
    % upper threshold. 1e-2 due to rounding upto 2nd decimals
    maxLevel = threshold(2) - cast(1e-2,dataType);
    dThresholdValues = [dBetweenLevels;maxLevel];
    
    allthreshold = unique(sort(dThresholdValues(:)));
    allowed = allthreshold > threshold(1) & allthreshold < threshold(2);
    % If there is no threshold between limits, use the limits to compute
    % two partitions.
    if sum(allowed) == 0
        partitionThreshold = threshold;
    else
        partitionThreshold = allthreshold(allowed);
    end
else
    partitionThreshold = threshold;
end

% compute partition for each threshold
numPartitions = min(numel(partitionThreshold),maxNumPartitions);
partitions = zeros(numel(detections),numPartitions,'uint32');

% Clusters found per epsilon in the loop
nFound = zeros(numPartitions,1,'uint32');

for i = 1:numel(uqSensors)
    thisSensor = sensorIdx == uqSensors(i);
    if calculateThreshold
        % Already calculated distance when finding thresholds, just use
        % that.
        distance = distanceMatrix(thisSensor,thisSensor);
    else
        % Compute ditance for these sensors
        thisSensorDetections = fusion.internal.selectDetections(detections, thisSensor);
        distance = fusion.internal.interDetectionDistance(thisSensorDetections,distanceType);
    end
    for k = 1:numPartitions
        currentThreshold = partitionThreshold(k);
        clusterIdx = fusion.internal.clusterUsingDistance(distance,currentThreshold);
        nNew = max(clusterIdx);
        partitions(thisSensor,k) = clusterIdx + nFound(k);
        nFound(k) = nFound(k) + nNew;
    end
end

% Return unique list
idx = zeros(1,numPartitions,'uint32');
[uqPartitions, ~, idx(:)] = unique(partitions','rows');
partitions = uqPartitions';

% Return second output when thresholds were user-supplied
if ~calculateThreshold
    varargout{1} = idx;
end

end



