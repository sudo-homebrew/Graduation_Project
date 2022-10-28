function [statesOut,costOut] =computeAssociationByIndex(...
    fusedStates,costValues,pDetections,stateEstimationFcn,measurementFcn,...
    minValidTriangulation,sizeValues,sampleBuffer,numSensors, Pd,Pfa,V, i)
% This is an internal function and may be removed or modified in the future

% Copyright 2018 The MathWorks, Inc.

%#codegen

[thisBuffer, detIndicators] = parseBuffer(...
    sampleBuffer, numSensors, pDetections, sizeValues, i);

% Coarse-gating - minimum 2 detections required in the tuple.
if numel(thisBuffer) >= minValidTriangulation
    
    [statesOut, costOut] = computeStateAndCost(...
        numSensors, thisBuffer, detIndicators, ...
        stateEstimationFcn, measurementFcn, ...
        Pd,Pfa,V);
    
else
    statesOut = fusedStates(:,i);
    costOut = costValues(i);
end
end

function logLikelihood = gaussLikelihood(res,S)
% Calculate log likelihood using gaussian distribution of r and S.
r = res(:);
logLikelihood = log(det(2*pi*S)) + r'/S*r;
end

function vecOutput = ind2subVecOutput(sizeValues,ndx,numDims)
k = cumprod(sizeValues);
vecOutput = zeros(numDims,1);
for j = numDims:-1:3
    vi = rem(ndx - 1,k(j-1)) + 1;
    vj = (ndx - vi)/k(j-1) + 1;
    vecOutput(j) = vj;
    ndx = vi;
end
vi = rem(ndx-1, sizeValues(1)) + 1;
vecOutput(1) = vi;
vecOutput(2) = (ndx - vi)/sizeValues(1) + 1;
end

function [thisBuffer, detIndicators] = parseBuffer(...
    sampleBuffer, numSensors, pDetections, sizeValues, i)

if coder.target('MATLAB')
    thisIndices = cell(numSensors,1);
    [thisIndices{:}] = ind2sub(sizeValues,i);
    thisIndices = [thisIndices{:}];
else
    thisIndices = ind2subVecOutput(sizeValues,i,numSensors);
end

thisBuffer = repmat({sampleBuffer{1}},[1 0]);
detIndicators = false(numSensors,1);
thisBufferLength = 0;
for k = 1:numSensors
    if thisIndices(k) > 1
        thisBuffer{end+1} = pDetections{k}{thisIndices(k) - 1};
        thisBufferLength = thisBufferLength + 1;
        detIndicators(k) = true;
    end
end
end

function [state, thisCost] = computeStateAndCost(...
    numSensors, thisBuffer, detIndicators, ...
    stateEstimationFcn, measurementFcn, ...
    Pd,Pfa,V)

coder.varsize('zEst',[6 1],[1 0]);

state = stateEstimationFcn(thisBuffer);
indLogLikelihood = zeros(numSensors,1,'like',state);

negCost = sum((detIndicators - 1).*log(1-Pd) - ...
    detIndicators.*log((Pd.*V)./Pfa));

if negCost < 0
    for m = 1:numel(thisBuffer)
        if ~isempty(thisBuffer{m})
            if isequal(measurementFcn,@posmeas)
                fullState = zeros(2*numel(state),1,'like',state);
                fullState(1:2:end) = state;
                zEst = cvmeas(fullState,thisBuffer{m}.MeasurementParameters);
            elseif isequal(measurementFcn,@velmeas)
                fullState = zeros(2*numel(state),1,'like',state);
                fullState(2:2:end) = state;
                zEst = cvmeas(fullState,thisBuffer{m}.MeasurementParameters);
            else
                zEst = measurementFcn(state,thisBuffer{m}.MeasurementParameters);
            end
            zAct = thisBuffer{m}.Measurement;
            r = zEst - zAct(:);
            S = thisBuffer{m}.MeasurementNoise;
            indLogLikelihood(m) = gaussLikelihood(r,S);
        end
    end
end
thisCost = negCost + sum(indLogLikelihood);
end

