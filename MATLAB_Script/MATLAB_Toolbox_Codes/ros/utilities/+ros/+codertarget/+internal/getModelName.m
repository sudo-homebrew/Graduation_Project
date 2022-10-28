function modelName = getModelName(hCS)
%This function is for internal use only. It may be removed in the future.

%getModelName Get name for current model
%   hCS is the Simulink.ConfigSet object.

%   Copyright 2016-2018 The MathWorks, Inc.

    if ~isempty(hCS.getModel)
        % In most cases, can get model name directly from SettingsController
        % object.
        modelName = get_param(hCS.getModel, 'Name');
    else
        % Configuration not owned by the model, directly query for the bdroot,
        % The model which invoked this code-path should get picked by bdroot.
        % This scenario can happen if reference configurations are used.
        modelName = bdroot;
    end

end
