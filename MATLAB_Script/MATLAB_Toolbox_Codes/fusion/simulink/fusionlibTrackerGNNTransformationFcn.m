function outData = fusionlibTrackerGNNTransformationFcn(inData)
% Cleanup obsolete properties from the instance data
%   This function is a transformation function for fusionlib specified in
%   the block forwarding table of fusionlib

%   Copyright 2019-2020 The MathWorks, Inc.

outData.NewBlockPath = ''; % forwarding table overrides the values defined in the transformation function
outData.NewInstanceData = [];

if ~isempty(inData) && isstruct(inData) && isfield(inData,'InstanceData') && ...
        isfield(inData.InstanceData,'Name')
    instanceData = inData.InstanceData;
    names = {instanceData.Name};
    
    instanceData = replaceParamNameAndValue(instanceData,names,'StateParams');
    outData.NewInstanceData = instanceData;
else
    outData = inData;
end

end

function instanceData = replaceParamNameAndValue(instanceData,names,paramName)
% Get the old parameter value, replaces it with the new parameter name and
% value.
isfound = contains(names,paramName);
if any(isfound)
    ind = find(isfound);    
    if strcmp(paramName,'StateParams')
        instanceData(ind).Name = 'StateParametersSimulink';% Value remains the same
    end
end
end