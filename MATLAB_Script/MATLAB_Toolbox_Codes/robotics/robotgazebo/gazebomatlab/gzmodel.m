function varargout = gzmodel(op, modelname, varargin)
%GZMODEL Assign and retrieve Gazebo model information
%   LIST = GZMODEL("list") returns and displays a list LIST of model names
%   available in the Gazebo world. If you do not define the output
%   argument, the model names are displayed in the MATLAB® Command Window.
%
%   [LINKS, JOINTS] = GZMODEL("info", MODELNAME) returns and displays a
%   list of link LINKS and joint JOINTS names of the specific MODELNAME. If
%   you do not define the output argument, the model info is displayed in
%   the MATLAB® Command Window.
%
%   [STATUS, MESSAGE] = GZMODEL("set", MODELNAME, NAME, VALUE) assigns
%   single or multiple values VALUE to the parameter with the name NAME for
%   the specified Gazebo model MODELNAME. The function returns STATUS as a
%   logical array and MESSAGE as a string array. The STATUS is true if the
%   parameter is set without any error. The MESSAGE returns details about
%   the success and failure. If you do not define the output argument, the
%   STATUS and MESSAGE details are displayed in the MATLAB® Command Window.
%
%       Name-Value:
%
%       EnableWind  - Enable or Disable Gazebo Model Wind Parameter with
%                     "on" or "off"
%       IsStatic    - Enable or Disable Gazebo Model IsStatic Parameter
%                     with "on" or "off"
%       Orientation - Set Gazebo Model Orientation as [ w, x, y, z] array
%       Position    - Set Gazebo Model Position as [ x, y, z] array
%       SelfCollide - Enable or Disable Gazebo Model SelfCollide Parameter
%                     with "on" or "off"
%
%   [OUTPUT1, ... ,OUTPUTN] = GZMODEL("get", MODELNAME, NAME) retrieves
%   single or multiple values VALUE of the parameter with the name NAME for
%   the specified Gazebo model MODELNAME. The function returns one or more
%   outputs as a scalar or array based on the specified parameter names. If
%   you do not define the output argument, the model parameter details are
%   displayed in the MATLAB® Command Window.
%
%       Name:
%
%       EnableWind  - Get Gazebo Model Wind Parameter status
%       IsStatic    - Get Gazebo Model IsStatic Parameter status
%       Orientation - Get Gazebo Model Orientation
%       Position    - Get Gazebo Model Position
%       SelfCollide - Get Gazebo Model SelfCollide Parameter status
%
%   SDFSTRING = GZMODEL("importSDF", MODELNAME) returns the Simulation
%   Description Format (SDF) of the specified Gazebo model as a string. If
%   you do not define the output argument, the model SDF details are
%   displayed in the MATLAB® Command Window.
%
%   Example I:
%      % Launch 'multiSensorPluginTest.world' from installed Gazebo server
%      % plugin before using following example
%
%      % List all model name available in Gazebo
%      list = GZMODEL("list");
%
%      % Get information about specific model
%      [links,joints] = GZMODEL("info","unit_box");
%
%      % Set Position and SelfCollide of Gazebo Model
%      [status,message] = GZMODEL("set","unit_box","Position",[2,2,0.5],"SelfCollide","on");
%
%      % Get Position and SelfCollide of Gazebo Model
%      [position,selfcollide] = GZMODEL("get","unit_box","Position","SelfCollide");
%
%   Example II:
%      % Launch 'jointLinkStatePluginTest.world' from installed Gazebo
%      % server plugin before using following example
%
%      % Get SDF details as string for specific Gazebo Model
%      sdfstring = GZMODEL("importSDF","two_wheel_vehicle");
%
%      % Import RigidBodyTree from the SDF string
%      robot = importrobot(sdfstring);
%
%      % Visualize RigidBodyTree
%      show(robot)
%
%      % Get RigidBodyTree details
%      showdetails(robot)
%
%   See also gzinit, gzworld, gzlink, gzjoint

%   Copyright 2020-2021 The MathWorks, Inc.

    narginchk(1,inf);

    op = convertStringsToChars(op);
    validateattributes(op, {'char'}, {'scalartext', 'nonempty'},...
                       'gzmodel', 'operation');

    if( nargin > 1)
        modelname = convertStringsToChars(modelname);

    end

    if nargin > 2
        [varargin{:}] = convertStringsToChars(varargin{:});
    end

    % Use validatestring to ensure string and char inputs to 'operation'
    % are supported.
    supportedOperations = {'list', 'info', 'set', 'get', 'importSDF'};
    try
        operation = validatestring(op, supportedOperations, 'gzmodel', 'operation');
    catch
        error(message('robotics:robotgazebo:gzsupport:InvalidOperation', op, ...
                      strjoin(supportedOperations,'","')));
    end

    switch(operation)
      case 'list'
        if( nargin > 1)
            error(message('robotics:robotgazebo:gzsupport:TooManyInputs',operation,1));
        end

        nargoutchk(0, 1);

        % get model info from Gazebo and retrieve model names
        modelOutput = robotics.gazebo.internal.MATLABInterface.utils.getModelList;

        if(strcmp(modelOutput,""))
            error(message('robotics:robotgazebo:gzsupport:EmptyGazebo'));
        else
            if nargout == 0
                % print model name list
                robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                    modelOutput,'v','MODEL LIST:');
            else

                varargout{1} = modelOutput;
            end
        end

      case 'info'
        if( nargin > 2)
            error(message('robotics:robotgazebo:gzsupport:TooManyInputs',operation,2));
        end

        nargoutchk(0, 2);

        validateattributes(modelname, {'char'}, {'scalartext', 'nonempty'},...
                           'gzmodel', 'modelname');

        % get model info from Gazebo and retrieve model names
        modelInfo = robotics.gazebo.internal.MATLABInterface.utils.getEntityList;
        modelList = [modelInfo.model_data.model_name];

        % find user input model name details
        idxList = strcmp(modelList,modelname);
        modelOutput  = modelInfo.model_data(idxList);

        if(isempty(modelOutput))
            error(message('robotics:robotgazebo:gzsupport:InvalidModelName',...
                          modelname,'gzmodel("list")'));
        else
            if nargout == 0
                % print model details
                printOutput =  modelOutput.model_name;
                robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                    printOutput,'h','MODEL:');

                printOutput =  modelOutput.links.link_name;
                robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                    printOutput,'h','LINKS:');

                printOutput =  modelOutput.joints.joint_name;
                robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                    printOutput,'h','JOINTS:');

            else
                varargout{1} = modelOutput.links.link_name;
                varargout{2} = modelOutput.joints.joint_name;
            end
        end

      case 'set'
        narginchk(4,inf);

        nargoutchk(0, 2);

        % validate model name
        validateattributes(modelname, {'char'}, {'scalartext', 'nonempty'},...
                           'gzmodel', 'modelname');

        % validate and add name-value pair inputs
        parser = robotics.gazebo.internal.MATLABInterface.utils.validateInputs(...
            'SetModel',  varargin{:});
        % set model parameter
        [status, errorMessage, printStatus, printMessage] = ...
            robotics.gazebo.internal.MATLABInterface.set.setModelParam(...
                modelname, parser, varargin{:});

        if nargout == 0
            robotics.gazebo.internal.MATLABInterface.utils.handleSetPrintOutput(...
                printStatus , printMessage);
        else
            varargout{1} = status;
            varargout{2} = errorMessage;
        end

      case 'get'
        narginchk(3,inf);

        validateattributes(modelname, {'char'}, {'scalartext', 'nonempty'},...
                           'gzmodel', 'modelname');

        % validate supported parameter
        supportedParameters = {'Position', 'Orientation', 'SelfCollide',...
                            'EnableWind','IsStatic'};

        validParamName = cell(1,numel(varargin));
        for vIdx = 1:numel(varargin)
            validParamName{vIdx} = validatestring(varargin{vIdx}, supportedParameters, 'gzmodel', 'parameter');
        end

        % get model parameter
        [result, resultString] = ...
            robotics.gazebo.internal.MATLABInterface.get.getModelParam(...
                modelname, validParamName);

        if nargout == 0
            robotics.gazebo.internal.MATLABInterface.utils.handleGetPrintOutput(...
                modelname, '', resultString, validParamName, 'getModel');
        else
            output = robotics.gazebo.internal.MATLABInterface.utils.handleGetReturnOutput(...
                result, validParamName, 'getModel');

            nargoutchk(0, numel(output));

            varargout = cell(1,nargout);
            for outIdx = 1:nargout
                varargout{outIdx} = output{outIdx};
            end
        end

      case 'importSDF'

        narginchk(2,2);

        nargoutchk(0, 1);

        validateattributes(modelname, {'char'}, {'scalartext', 'nonempty'},...
                           'gzmodel', 'modelname');

        % get model SDF string
        sdfstring = robotics.gazebo.internal.MATLABInterface.get.getModelSDFString(modelname);

        if nargout == 0
            % print SDF details
            robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                modelname,'h','MODEL:');

            robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                sdfstring,'h','SDFSTRING:');
        else
            varargout{1} = sdfstring;
        end

    end

end
