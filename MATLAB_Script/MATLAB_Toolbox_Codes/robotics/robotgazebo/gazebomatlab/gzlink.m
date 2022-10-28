function varargout = gzlink(op, modelname, linkname, varargin)
%GZLINK Assign and retrieve Gazebo model link information
%   LIST = GZLINK("list", MODELNAME) returns and displays a list LIST of
%   link names of the specific MODELNAME. If you do not define the output
%   argument, the model link names are displayed in the MATLAB® Command
%   Window.
%
%   [STATUS, MESSAGE] = GZLINK("set", MODELNAME, LINKNAME, NAME, VALUE)
%   assigns single or multiple values VALUE to the parameter with the name
%   NAME for the specified Gazebo model MODELNAME and link LINKNAME. The
%   function returns STATUS as a logical array and MESSAGE as a string
%   array. The STATUS is true if the parameter is set without any error.
%   The MESSAGE returns details about the success and failure. If you do
%   not define the output argument, the STATUS and MESSAGE details are
%   displayed in the MATLAB® Command Window.
%
%       Name-Value:
%
%       Canonical        - Enable or Disable Gazebo Model-Link Canonical
%                          Parameter with "on" or "off"
%       EnableWind       - Enable or Disable Gazebo Model-Link Wind
%                          Parameter with "on" or "off"
%       Gravity          - Enable or Disable Gazebo Model-Link Gravity
%                          Parameter with "on" or "off"
%       IsStatic         - Enable or Disable Gazebo Model-Link IsStatic
%                          Parameter with "on" or "off"
%       Kinematic        - Enable or Disable Gazebo Model-Link Kinematic
%                          Parameter with "on" or "off"
%       Mass             - Set Gazebo Model-Link Mass as a scalar value
%       Orientation      - Set Gazebo Model-Link Orientation as
%                          [ w, x, y, z] array
%       Position         - Set Gazebo Model-Link Position as [ x, y, z]
%                          array
%       PrincipalMoments - Set Gazebo Model-Link Principal Moments as
%                          [ ixx, iyy, izz] array
%       ProductOfInertia - Set Gazebo Model-Link Product of Inertia as
%                          [ ixy, ixz, iyz] array
%       SelfCollide      - Enable or Disable Gazebo Model-Link SelfCollide
%                          Parameter with "on" or "off"
%
%   [OUTPUT1, ... ,OUTPUTN] = GZLINK("get", MODELNAME, LINKNAME, NAME)
%   retrieves single or multiple values VALUE of the parameter with the
%   name NAME for the specified Gazebo model MODELNAME and link LINKNAME.
%   The function returns one or more outputs as a scalar or array
%   based on the specified parameter names. If you do not define the output
%   argument, the model link parameter details are displayed in the
%   MATLAB® Command Window.
%
%       Name:
%
%       Canonical        - Get Gazebo Model-Link Canonical Parameter status
%       EnableWind       - Get Gazebo Model-Link Wind Parameter status
%       Gravity          - Get Gazebo Model-Link Gravity Parameter status
%       IsStatic         - Get Gazebo Model-Link IsStatic Parameter status
%       Kinematic        - Get Gazebo Model-Link Kinematic Parameter status
%       Mass             - Get Gazebo Model-Link Mass
%       Orientation      - Get Gazebo Model-Link Orientation
%       Position         - Get Gazebo Model-Link Position
%       PrincipalMoments - Get Gazebo Model-Link Principal Moments
%       ProductOfInertia - Get Gazebo Model-Link Product of Inertia
%       SelfCollide      - Get Gazebo Model-Link SelfCollide Parameter
%                          status
%
%   Example:
%      % Launch 'multiSensorPluginTest.world' from installed Gazebo server
%      % plugin before using following example
%
%      % List all link names of specific model available in Gazebo
%      list = GZLINK("list","unit_box");
%
%      % Set Position and Gravity of Gazebo Model Link
%      [status,message] = GZLINK("set","unit_box","link","Position",[2,2,0.5],"Gravity","off");
%
%      % Get Position and Gravity of Gazebo Model Link
%      [position,gravity] = GZLINK("get","unit_box","link","Position","Gravity");
%
%   See also gzinit, gzworld, gzmodel, gzjoint

%   Copyright 2020-2021 The MathWorks, Inc.

    narginchk(1,inf);

    op = convertStringsToChars(op);
    validateattributes(op, {'char'}, {'scalartext', 'nonempty'},...
                       'gzlink', 'operation');

    if( nargin > 1)
        modelname = convertStringsToChars(modelname);
        validateattributes(modelname, {'char'}, {'scalartext', 'nonempty'},...
                           'gzlink', 'modelname');
    end

    if( nargin > 2)
        linkname = convertStringsToChars(linkname);
    end

    if nargin > 3
        [varargin{:}] = convertStringsToChars(varargin{:});
    end

    % Use validatestring to ensure string and char inputs to 'operation' are
    % supported.
    supportedOperations = {'list', 'set', 'get'};
    try
        operation = validatestring(op, supportedOperations, 'gzmodel', 'operation');
    catch
        error(message('robotics:robotgazebo:gzsupport:InvalidOperation', op, ...
                      strjoin(supportedOperations,'","')));
    end

    switch(operation)
      case 'list'
        if( nargin > 2)
            error(message('robotics:robotgazebo:gzsupport:TooManyInputs',operation,2));
        end
        narginchk(2,2);

        nargoutchk(0, 1);

        % get model info from Gazebo and retrieve model names
        modelInfo = robotics.gazebo.internal.MATLABInterface.utils.getEntityList;
        modelList = [modelInfo.model_data.model_name];

        % find user input model name details
        idxList = strcmp(modelList,string(modelname));

        if(~nnz(idxList))
            error(message('robotics:robotgazebo:gzsupport:InvalidModelName',...
                          modelname,'gzmodel("list")'));
        end

        linkOutput =  modelInfo.model_data(idxList).links.link_name;

        if nargout == 0
            % print list of model link names
            printOutput =  modelInfo.model_data(idxList).model_name;
            robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                printOutput,'h','MODEL:');

            robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                linkOutput,'h','LINKS:');
        else
            % returns link list
            varargout{1} = linkOutput;
        end

      case 'set'
        narginchk(5,inf);

        nargoutchk(0, 2);

        % validate link name
        validateattributes(linkname, {'char'}, {'scalartext', 'nonempty'},...
                           'gzlink', 'linkname');

        % validate and add name-value pair inputs
        parser = robotics.gazebo.internal.MATLABInterface.utils.validateInputs(...
            'SetLink', varargin{:});
        % set model-link parameter
        [status, errorMessage, printStatus, printMessage]  = ...
            robotics.gazebo.internal.MATLABInterface.set.setLinkParam(...
                modelname, linkname, parser, varargin{:});

        if nargout == 0
            robotics.gazebo.internal.MATLABInterface.utils.handleSetPrintOutput(...
                printStatus , printMessage);
        else
            varargout{1} = status;
            varargout{2} = errorMessage;
        end

      case 'get'
        narginchk(4,inf);

        validateattributes(linkname, {'char'}, {'scalartext', 'nonempty'},...
                           'gzlink', 'linkname');

        % validate supported parameter
        supportedParameters = {'Position', 'Orientation','Mass',...
                            'ProductOfInertia','PrincipalMoments','IsStatic',...
                            'Canonical','SelfCollide','EnableWind','Gravity',...
                            'Kinematic'};

        validParamName = cell(1,numel(varargin));
        for vIdx = 1:numel(varargin)
            validParamName{vIdx} = validatestring(varargin{vIdx}, supportedParameters, 'gzlink', 'parameter');
        end

        % get model-link parameter
        [result, resultString] = ...
            robotics.gazebo.internal.MATLABInterface.get.getLinkParam(...
                modelname, linkname, validParamName);

        if nargout == 0
            robotics.gazebo.internal.MATLABInterface.utils.handleGetPrintOutput(...
                modelname, linkname, resultString, validParamName, 'getLink');
        else
            output = robotics.gazebo.internal.MATLABInterface.utils.handleGetReturnOutput(...
                result, validParamName, 'getLink');

            nargoutchk(0, numel(output));

            varargout = cell(1,nargout);
            for outIdx = 1:nargout
                varargout{outIdx} = output{outIdx};
            end
        end
    end

end
