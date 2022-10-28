function paramName = storeParamInDataDictionary(blkHandle,paramName,paramVal)
%   Store a parameter with name paramName and value paramVal in a data
%   dictionary.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

%Find root level model name.
model = bdroot(get_param(blkHandle,'Parent'));

%Check if the model already has a dictionary associated with it.
dictionaryName = get_param(model,'DataDictionary');

if isempty(dictionaryName)
    %Data dictionary does not exist, create a new one.
    dictionaryName = getUniqueDictionaryName(model);
    dictObj = Simulink.data.dictionary.create(dictionaryName);
    % Link the dictionary to the model.
    set_param(model,'DataDictionary',dictionaryName);
else
    %Data dictionary already exists. Open existing dictionary.
    dictObj = Simulink.data.dictionary.open(dictionaryName);
end

if Simulink.data.existsInGlobal(model,paramName)
    %A variable with same name already exists in base workspace, 
    % or in data dictionary.
    % Get a unique param name.
    paramName = getUniqueParameterName(model,paramName);
end

%Store the data in dictionary
Simulink.data.assigninGlobal(model,paramName,paramVal);
% Save changes to the dictionary and close it.
saveChanges(dictObj);
close(dictObj);
end

function uniqueName = getUniqueParameterName(model,paramName)
%Creates a unique parameter name by appending a unique number to paramName.
notUnique = true;
prefix = paramName;
idx = 1;
while notUnique
    %Check if a parameter with the same name already exist in dictionary or
    %base workspace.
    uniqueName = [prefix num2str(idx)];
    notUnique = Simulink.data.existsInGlobal(model,uniqueName);
    idx = idx+1;

end
end

function uniqueName = getUniqueDictionaryName(modelName)
%Creates a unique dictionary name with a combination of model name and a
%unique number.
notUnique = true;
prefix = [modelName,'_ModelData'];
idx = 1;
while notUnique
    uniqueName = [prefix num2str(idx) '.sldd'];
    %Check if the dictionary with name uniqueName already exists.
    notUnique = exist(uniqueName,'file');
    idx = idx+1;
end
end