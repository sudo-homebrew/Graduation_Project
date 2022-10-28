function [fusedStates,costMatrix] = calcNDCostMatrix(...
    fusedStates,costValues,pDetections,stateEstimationFcn,measurementFcn,...
    minValidTriangulation,sizeValues,sampleBuffer,Pd,Pfa,V)
% This is an internal function and may be removed or modified in a future
% release.
%
% This function calculates the N-Dimensional cost matrix and correspoding
% fused measurements for static detection fusion.

% Copyright 2018 The MathWorks, Inc.

%#codegen
numAssociations = size(fusedStates,2);
numSensors = numel(pDetections);

for i = 1:numAssociations
    
    [fusedStatesOuti,costValuesOuti] = ...
        fusion.internal.computeAssociationByIndex(...
        fusedStates,costValues,pDetections,stateEstimationFcn,measurementFcn,...
        minValidTriangulation,sizeValues,sampleBuffer,numSensors, Pd,Pfa,V, i);

    fusedStates(:,i) = fusedStatesOuti;
    costValues(i) = costValuesOuti;

end

% Fine-gating of scores. We can set it to inf to permanently gate them.
% Note that one must not set the cost of false alarms/missed detections as
% inf. They are set to 0 in this case.
costValues(costValues>0) = inf;
% Assemble cost values in a cost matrix.
costMatrix = zeros(sizeValues,'like',costValues);
costMatrix(:) = costValues;

end
