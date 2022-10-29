function [partitions, ic] = partitionDBSCAN(detections, varargin)
% This is an internal function and may be removed or modified in a future
% release.
%
% partitions = partitionDBSCAN(detections, epsilon, minNumPts) computes the
% partitions of a detection set detections for a given vector of distance
% thresholds (epsilons). The function uses Mahalanobis distance between
% detections by default.
%
% partitions = partitionDBSCAN(detections, ..., 'Distance', distanceType)
% allows you to specify the Distance as a N/V pair. The available choices
% are "Euclidean" and "Mahalanobis"
%
% detections is a cell array of objectDetection objects
%
% [partitions, idx] = partitionDBSCAN(..) provides a mapping between
% epsilon value and unique partitions. 

% Copyright 2021 The MathWorks, Inc.

%#codegen

% Parse inputs
[epsilon, minNumPts, distType] = fusion.internal.parsePartitionDBSCANInputs(detections, varargin{:});

if isempty(detections)
    partitions = zeros(0,1,'uint32');
    ic = ones(size(epsilon),'uint32');
    return;
end
% Allocate memory for partitions
partitions = zeros(numel(detections),numel(epsilon),'uint32');

% Each detection's SensorIndex
sensorIdx = fusion.internal.concatenateDetectionData(detections,'SensorIndex');

% Partition per Sensor
uqSensors = unique(sensorIdx);

% Clusters found per epsilon in the loop
nFound = zeros(numel(epsilon),1,'uint32');

for i = 1:numel(uqSensors)
    thisSensor = sensorIdx == uqSensors(i);
    thisSensorDetections = fusion.internal.selectDetections(detections, thisSensor);
    distance = fusion.internal.interDetectionDistance(thisSensorDetections,distType);
    for k = 1:numel(epsilon)
        clusterIdx = fusion.internal.DBSCAN.clusterUsingDistance(distance,epsilon(k),minNumPts(k));
        nNew = max(0,max(clusterIdx));
        partitions(thisSensor,k) = clusterIdx + nFound(k);
        nFound(k) = nFound(k) + nNew;
    end
end

ic = zeros(1,numel(epsilon),'uint32');
[uqPartitions, ~, ic(:)] = unique(partitions','rows');
partitions = uqPartitions';

end