function setBlockMaskParamsFromObject(blkHandle,matlabObj,params)
% Set block mask parameters params from matlabObj to the block with handle
% blkHandle .
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.


%Setting individual params with set_param is slow as it triggers property
%validation every time a parameter is set. It is much more efficient to set
%multiple params with a single call to set_params. Setting parameters
%individually is also an issue when properties are interdependent and need
%to be set together before validation is performed.

%Creating an expression to set the parameters with a single call.
setParamsExp = strcat('set_param(','blkHandle',',');
for i = 1:numel(params)
    if islogical(matlabObj.(params{i}))
        %Check if the property is also logical in matlabObj.
        if isLogicalProperty(matlabObj,params{i})
            setParamsExp = strcat(setParamsExp,'''',params{i},'''',',',...
                '''',char(matlab.lang.OnOffSwitchState(matlabObj.(params{i}))),'''',',');
        else
            setParamsExp = strcat(setParamsExp,'''',params{i},'''',',',...
                '''',mat2str(matlabObj.(params{i})),'''',',');
        end
    elseif ischar(matlabObj.(params{i})) || isstring(matlabObj.(params{i}))
        setParamsExp = strcat(setParamsExp,'''',params{i},'''',','...
            ,'''',char(matlabObj.(params{i})),'''',',');
    elseif isnumeric(matlabObj.(params{i}))
        setParamsExp = strcat(setParamsExp,'''',params{i},'''',','...
            ,'''',mat2str(matlabObj.(params{i})),'''',',');
    elseif isa(matlabObj.(params{i}),'function_handle')
        setParamsExp = strcat(setParamsExp,'''',params{i},'''',','...
            ,'''',func2str(matlabObj.(params{i})),'''',',');
    elseif isstruct(matlabObj.(params{i})) || iscell(matlabObj.(params{i}))
        storedParam = fusion.simulink.internal.addParamInModelPreLoadCallback...
            (blkHandle,params{i},matlabObj.(params{i}));
        setParamsExp = strcat(setParamsExp,'''',params{i},'''',',',...
            '''',storedParam,'''',',');
    else
        %Parameters are set as char vectors, so if an unknown datatype is
        %encountered, generate a warning indicating that the parameter is
        %not set.
        blkName = get_param(blkHandle,'Name');
        warning(message('fusion:simulink:exportToSimulink:FailedToSetParam',params{i},blkName,class(matlabObj)));
    end
end
setParamsExp=strcat(setParamsExp(1:end-1),')');
eval(setParamsExp);
end

function flag = isLogicalProperty(obj,prop)
%Find if a property prop is logical in an object obj.

%Metadata about the class.
info = metaclass(obj);
flag = any(arrayfun(@(x)strcmpi(x.Name,prop) && ...
    (x.Logical || ...                  % legacy attribute: Logical
    (length(x.Validation) == 1 && ...  % logical validator; similar logic in convertToTrueFalseStructPropertyGroups.m
      ~isempty(x.Validation.Class) && ...
      strcmp(x.Validation.Class.Name, 'logical'))), ...
    info.PropertyList));
end
