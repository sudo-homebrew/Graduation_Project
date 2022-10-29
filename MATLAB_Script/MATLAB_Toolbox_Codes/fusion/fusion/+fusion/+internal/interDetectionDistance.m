function dMatrix = interDetectionDistance(detections, distType)
% This is an internal function and may be removed or modified in a future
% release.
%
% dMatrix = interDetectionDistance(detections) calculates the Mahalanobis
% distance between detections.
% dMatrix = interDetectionDistance(detections, distanceType) allows you
% to specify the distance type. Available choices are 'Mahalanobis' and
% 'Euclidean'
%
% detections is a N-element cell array of objectDetection objects.
% dMatrix is a NxN matrix, where N is the number of detections.

% Copyright 2018-2021 The MathWorks, Inc.

%#codegen

if nargin == 1
    distType = 'Mahalanobis';
end

switch lower(distType)
    case 'euclidean'
        dMatrix = euclideanDistance(detections);
    case 'mahalanobis'
        dMatrix = mahalanobisDistance(detections);
    otherwise
        assert(false);
end

end

function dMatrix = euclideanDistance(detections)
x = fusion.internal.concatenateDetectionData(detections,'Measurement');
% Vectorized Euclidean distance calculation
xTx = dot(x,x,1);
dMatrix = bsxfun(@plus, xTx , xTx') - 2*(x'*x);
n = numel(detections);
% Resolve diagonal to be zero for resolving small errors due to above
% calculation
idx = 1:(n + 1):n^2;
dMatrix(idx) = 0;
dMatrix(dMatrix < 0) = 0;
dMatrix = sqrt(dMatrix);
end

function dMatrix = mahalanobisDistance(detections)
classToUse = class(detections{1}.Measurement);
shortCalculation = isShortCalculationPossible(detections);

n = numel(detections);
dMatrix = zeros(n,classToUse);

% short calculation. In the short calculation each detection is evaluated
% against other detections by computing its errors and using a covariance
% of 2 x MeasurementNoise.
if shortCalculation
    R = 2*detections{1}.MeasurementNoise;
    mAll = fusion.internal.concatenateDetectionData(detections,'Measurement');
    for i = 1:numel(detections)-1
        m1 = mAll(:,i);
        mOthers = mAll(:,i+1:end);
        e = bsxfun(@minus,m1,mOthers);
        eTRinv = e'/R;
        d = sqrt(dot(eTRinv,e',2));
        dMatrix(i,i+1:end) = d;
        dMatrix(i+1:end,i) = d;
    end
    % long calculation. In the long calculation, distance between each
    % detection is computed pair-wise.
else
    for i = 1:n
        for j = i+1:n
            dMatrix(i,j) = distanceOneToOne(detections{i},detections{j});
            dMatrix(j,i) = dMatrix(i,j);
        end
    end
end
end

function d = distanceOneToOne(det1,det2)
% calculates the mahalanobis distance between two detections.
m1 = det1.Measurement(:);
m2 = det2.Measurement(:);
R1 = det1.MeasurementNoise;
R2 = det2.MeasurementNoise;
S = R1 + R2;
e = m1 - m2;
d = sqrt(e'/S*e);
end

function shortCalculation = isShortCalculationPossible(detections)
% If measurement noise is almost same, do short calculation.
shortCalculation = false;
R1 = detections{1}.MeasurementNoise;
eigR1 = eig(R1);
for i = 2:numel(detections)
    R = detections{i}.MeasurementNoise;
    if all(abs(eig(R) - eigR1) < 0.1)
        shortCalculation = true;
    else
        shortCalculation = false;
        break;
    end
end
end
