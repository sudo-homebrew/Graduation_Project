function model = initializeSimulinkModel(varargin)
%   Create and initialize a Simulink model.
%
%   This function is for internal use only and may be removed in a future
%   release.

%   Copyright 2021 The MathWorks, Inc.

if isempty(varargin)
    % Model name is not provided, create a new model and return the name.
    h = new_system();
    load_system(h);
    model = get_param(h,'Name');
else
    if isnumeric(varargin{1})
        % Model handle is provided, load the model and return the model
        % name. if a valid model handle is not provided, the function
        % generates appropriate error message.
        h = varargin{1};
        try
            load_system(h);
        catch ME
            if strcmpi(ME.identifier,'Simulink:Commands:InvSimulinkObjHandle')
                error(message('fusion:simulink:exportToSimulink:InvalidModelHandle',h));               
            else
                rethrow(ME);
            end
        end
        model = get_param(h,'Name');
    else
        % Model name is provided.
        model = char(varargin{1});
        try
            %load the model.
            load_system(model);
        catch
            % Model does not exist, create a new one
            new_system(model);
        end
    end
end
end