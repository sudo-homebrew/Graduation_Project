function [calculateThreshold, threshold, maxNumPartitions, distanceType, classToUse] = parsePartitionDPInputs(detections, varargin)
% This is an internal function and may be removed or modified in a future
% release.

% This function parses the inputs to partitionDetections when algorithm is
% Distance-Partitioning.

% Copyright 2021 The MathWorks, Inc.

%#codegen

% function name for errors
funcName = 'partitionDetections';

% Validate elements of the cell array
validateattributes(detections{1},{'objectDetection','struct'},{'vector'},funcName,'detections{:}',1);

% Class to use
classToUse = class(detections{1}.Measurement);

% Parse inputs
opArgs = {'FirstInput','SecondInput'};
poptions = struct('CaseSensitivity', true,...
    'IgnoreNulls',true,...
    'SupportOverrides', false);
NVPairNames = {'Distance','MaxNumPartitions'};
pstruct = coder.internal.parseInputs(opArgs,NVPairNames,...
    poptions,varargin{:});

% Default distance is Mahalanobis
defaultDistType = 'Mahalanobis';
distanceTypeIn = coder.internal.getParameterValue(pstruct.Distance,defaultDistType,varargin{:});

if ~coder.target('MATLAB')
    distanceType = validatestring(distanceTypeIn,{'Mahalanobis','Euclidean'},'Algorithm');
else
    distanceType = localDistValidateString(lower(distanceTypeIn));
end

% Default 100 partitions if tLower, tUpper is provided
defaultMaxNumPartitions = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntIndex(100);
maxNumPartitionsIn = coder.internal.getParameterValue(pstruct.MaxNumPartitions,defaultMaxNumPartitions,varargin{:});

% If second input is provided, thresholds are provided as lower, upper
if pstruct.FirstInput > 0 && pstruct.SecondInput > 0
    tLower = varargin{pstruct.FirstInput};
    tUpper = varargin{pstruct.SecondInput};
    % Validate lower threshold
    validateattributes(tLower,{classToUse},...
        {'real','finite','nonsparse','scalar'},funcName,'Lower Threshold',2);
    
    % Validate upper threshold
    validateattributes(tUpper,{classToUse},...
        {'real','finite','nonsparse','scalar'},funcName,'Upper Threshold',3);
    
    % Send to parser output
    threshold = [tLower tUpper];
    calculateThreshold = true;
    validateattributes(maxNumPartitionsIn,{'numeric'},...
        {'real','finite','nonsparse','scalar','integer'},funcName,'MaxNumPartitions');
    maxNumPartitions = cast(maxNumPartitionsIn,'like',defaultMaxNumPartitions);
elseif pstruct.FirstInput > 0 && pstruct.SecondInput == 0 % Only first input
    % Array of thresholds
    threshold = varargin{pstruct.FirstInput};
    validateattributes(threshold,{classToUse},...
        {'real','finite','nonsparse','vector'},funcName,'Thresholds',2);
    calculateThreshold = false;
    maxNumPartitions = matlabshared.tracking.internal.fusion.codegen.StrictSingleUtilities.IntNumel(threshold);
else % No threshold inputs
    % Use default threshold range
    threshold = [cast(0.25,classToUse) cast(6.25,classToUse)];
    calculateThreshold = true;
    validateattributes(maxNumPartitionsIn,{'numeric'},...
        {'real','finite','nonsparse','scalar','integer'},funcName,'MaxNumPartitions');
    maxNumPartitions = cast(maxNumPartitionsIn,'like',defaultMaxNumPartitions);
end

end

function alg = localDistValidateString(alg)
arguments %#ok<EMFIV> 
    alg {mustBeMember(alg,{'mahalanobis','euclidean'})}
end
end