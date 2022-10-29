function blkHandle = addBlockAndSetParameters(matlabObj,varargin)
% Add a block equivalent to matlabObj in a Simulink model, set block mask
% parameters and return the numeric handle to the block.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

%Add block to the model.
try
    blkHandle = fusion.simulink.internal.addBlockToModel(matlabObj,varargin{:});
catch ME 
    % If an exception is generated while adding a block, add a cause to the
    % underlying exception and rethrow as caller.
    causeException = MException(message('fusion:simulink:exportToSimulink:FailedToExportException',class(matlabObj)));   
    ME = addCause(ME,causeException);
    throwAsCaller(ME);
end

% Populate properties that are active.
props = properties(matlabObj);
props = props(cellfun(@(x)~matlabObj.isInactiveProperty(x),props));

% Populate block mask parameters
blkParams = get_param(blkHandle,'MaskNames');

%Set parameters that are common between MATLAB object and
%Simulink block.
props = intersect(props,blkParams);

%Set parameters on block mask.
fusion.simulink.internal.setBlockMaskParamsFromObject(blkHandle,matlabObj,props);

if any(ismember(blkParams,'StateParametersSimulink'))
    %StateParameters property is exposed as StateParametersSimulink in
    %Simulink.
    set_param(blkHandle,'StateParametersSimulink',...
       fusion.simulink.internal.addParamInModelPreLoadCallback(blkHandle,'StateParameters',matlabObj.StateParameters));
end
end