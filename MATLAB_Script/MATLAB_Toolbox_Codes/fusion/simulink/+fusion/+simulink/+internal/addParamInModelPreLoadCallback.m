function paramName = addParamInModelPreLoadCallback(blkHandle,paramName,paramVal)
%   Adds an expression in model 'PreloadFcn' callback, so that whenever the
%   model is opened, required variables are created in base workspace.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

%Find root level model name.
model = bdroot(get_param(blkHandle,'Parent'));

% create a readable expression that can be evaluated.
stExp = fusion.simulink.internal.structToExpression(paramVal);

if strcmpi(stExp,'struct')
    %paramVal is just an empty struct, no need to create a variable.
    paramName = stExp;
else
    if Simulink.data.existsInGlobal(model,paramName)
        %A variable with same name already exists in base workspace,
        % or in data dictionary. Get a unique param name.
        paramName = getUniqueParameterName(model,paramName);
    end
    %Get existing commands in PreloadFcn.
    preloadCommand = get_param(model,'PreloadFcn');
    %Append in existing command.
    preloadCommand = append(preloadCommand, newline, newline, paramName, ' = ', stExp, ';');
    %Update the PreloadFcn callback.
    set_param(model,'PreloadFcn',preloadCommand);
    %Assign the variable in base workspace for current session.
    Simulink.data.assigninGlobal(model,paramName,paramVal);

    %Add a CloseFcn callback to clear variable from workspace.    
    %Get existing commands in CloseFcn.   
    closeCommand = get_param(model,'CloseFcn');
    %Append in existing command.
    closeCommand = append(closeCommand, newline, newline, 'clear ', paramName, ';');
    %Update the CloseFcn callback.
    set_param(model,'CloseFcn',closeCommand);
end
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