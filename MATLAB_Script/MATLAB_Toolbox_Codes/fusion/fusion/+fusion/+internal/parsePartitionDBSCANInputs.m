function [epsilon, minNumPts, distanceType] = parsePartitionDBSCANInputs(detections, varargin)
% This is an internal function and may be removed or modified in a future
% release.

% This function parses the inputs to partitionDetections when algorithm is
% DBSCAN.

% Copyright 2021 The MathWorks, Inc.

%#codegen

% Error throwing function
funcName = 'partitionDetections';

validateattributes(detections{1},{'objectDetection','struct'},{'vector'},funcName,'detections{:}',1);

% Class to use
classToUse = class(detections{1}.Measurement);

% Parse inputs
opArgs = {'Epsilon','MinNumPts'};
poptions = struct('CaseSensitivity', true,...
    'IgnoreNulls',true,...
    'SupportOverrides', false);
NVPairNames = {'Distance','Algorithm'};
pstruct = coder.internal.parseInputs(opArgs,NVPairNames,...
    poptions,varargin{:});

% Default epsilon is linearly spaced between 0.5 and 6.25
defaultEpsilon = linspace(cast(0.5,classToUse),cast(6.25,classToUse),10);

% Default points are 3
defaultMinNumPts = 3;

% Default distance is Mahalanobis
defaultDistType = 'Mahalanobis';

% Get value from parser
epsilon = coder.internal.getParameterValue(pstruct.Epsilon,defaultEpsilon,varargin{:});
validateattributes(epsilon,{classToUse},{'real','finite','nonsparse','vector'},funcName,'epsilon');

minNumPtsIn = coder.internal.getParameterValue(pstruct.MinNumPts,defaultMinNumPts,varargin{:});
validateattributes(minNumPtsIn,{'numeric'},{'real','finite','nonsparse','vector','integer','positive'},funcName,'minNumPts');

distanceTypeIn = coder.internal.getParameterValue(pstruct.Distance,defaultDistType,varargin{:});
if coder.target('MATLAB')
    distanceType = localDistValidateString(lower(distanceTypeIn));
else
    distanceType = validatestring(distanceTypeIn,{'Mahalanobis','Euclidean'},'Algorithm');
end

% Scalar expand MinNumPts
if isscalar(minNumPtsIn)
    minNumPts = repmat(minNumPtsIn,1,numel(epsilon));
else
    minNumPts = minNumPtsIn;
end

end

function alg = localDistValidateString(alg)
arguments %#ok<EMFIV>
    alg {mustBeMember(alg,{'mahalanobis','euclidean'})}
end
end