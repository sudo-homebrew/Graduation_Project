function varargout = gzjoint(op, modelname, jointname, varargin)
%GZJOINT Assign and retrieve Gazebo model joint information
%   LIST = GZJOINT("list", MODELNAME) returns and displays a list LIST of
%   joint names of the specific MODELNAME. If you do not define the output
%   argument, the model joint names are displayed in the MATLAB® Command
%   Window.
%
%   [STATUS, MESSAGE] = GZJOINT("set", MODELNAME, JOINTNAME, NAME, VALUE)
%   assigns single or multiple values VALUE to the parameter with the name
%   NAME for the specified Gazebo model MODELNAME and joint JOINTNAME. The
%   function returns STATUS as a logical array and MESSAGE as a string
%   array. The STATUS is true if the parameter is set without any error.
%   The MESSAGE returns details about the success and failure. If you do
%   not define the output argument, the STATUS and MESSAGE details are
%   displayed in the MATLAB® Command Window.
%
%       Name-Value:
%
%       Axis          - Select Axis index of Gazebo Model-Joint as "0" or
%                       "1"
%
%                       Option :
%                           Angle    - Set Angle parameter of Gazebo
%                                      Model-Joint Axis as a scalar value
%                           Damping  - Set Damping parameter of Gazebo
%                                      Model-Joint Axis as a scalar value
%                           Friction - Set Friction parameter of Gazebo
%                                      Model-Joint Axis as a scalar value
%                           XYZ      - Set XYZ values of Gazebo Model-Joint
%                                      Axis as [ X, Y, Z] array
%
%       CFM           - Set CFM parameter of Gazebo Model-Joint as a
%                       scalar value
%       FudgeFactor   - Set Fudge Factor parameter of Gazebo Model-Joint
%                       as a scalar value
%       Orientation   - Set Gazebo Model-Joint Orientation as [ w, x, y, z]
%                       array
%       Position      - Set Gazebo Model-Joint Position as [ x, y, z] array
%       SuspensionCFM - Set Suspension CFM parameter of Gazebo Model-Joint
%                       as a scalar value
%       SuspensionERP - Set Suspension ERP parameter of Gazebo Model-Joint
%                       as a scalar value
%
%   [OUTPUT1, ... ,OUTPUTN] = GZJOINT("get", MODELNAME, JOINTNAME, NAME)
%   retrieves single or multiple values VALUE of the parameter with the
%   name NAME for the specified Gazebo model MODELNAME and joint JOINTNAME.
%   The function returns one or more outputs as a scalar or array
%   based on the specified parameter names. If you do not define the output
%   argument, the model joint parameter details are displayed in the
%   MATLAB® Command Window.
%
%       Name:
%
%       Axis0, Axis1  - Select Axis0 or Axis1 to get Axis parameter,
%                       depending on availability of Axis of Gazebo
%                       Model-Joint
%
%                       Option :
%                           Angle    - Get Angle parameter of Gazebo
%                                      Model-Joint for Axis0 or Axis1
%                           Damping  - Get Damping parameter of Gazebo
%                                      Model-Joint for Axis0 or Axis1
%                           Friction - Get Friction parameter of Gazebo
%                                      Model-Joint for Axis0 or Axis1
%                           XYZ      - Get XYZ values of Gazebo Model-Joint
%                                      for Axis0 or Axis1
%
%       CFM           - Get CFM parameter of Gazebo Model-Joint
%       FudgeFactor   - Get Fudge Factor parameter of Gazebo Model-Joint
%       Orientation   - Get Gazebo Model-Joint Orientation
%       Position      - Get Gazebo Model-Joint Position
%       SuspensionCFM - Get Suspension CFM parameter of Gazebo Model-Joint
%       SuspensionERP - Get Suspension ERP parameter of Gazebo Model-Joint
%
%   Example:
%      % Launch 'multiSensorPluginTest.world' from installed Gazebo server
%      % plugin before using following example
%
%      % List all joint names of specific model available in Gazebo
%      list = GZJOINT("list","unit_box");
%
%      % Set Position and Damping of Gazebo Model Joint
%      [status,message] = GZJOINT("set","unit_box","joint","Position",[2,2,0.5],"Axis","0","Damping",0.25);
%
%      % Get Position and Damping of Gazebo Model Joint
%      [position,damping] = GZJOINT("get","unit_box","joint","Position","Axis0","Damping");
%
%   See also gzinit, gzworld, gzmodel, gzlink

%   Copyright 2020-2021 The MathWorks, Inc.

    narginchk(1,inf);

    op = convertStringsToChars(op);
    validateattributes(op, {'char'}, {'scalartext', 'nonempty'},...
                       'gzjoint', 'operation');

    if( nargin > 1)
        modelname = convertStringsToChars(modelname);
        validateattributes(modelname, {'char'}, {'scalartext', 'nonempty'},...
                           'gzjoint', 'modelname');
    end

    if( nargin > 2)
        jointname = convertStringsToChars(jointname);
    end

    if (nargin > 3)
        [varargin{:}] = convertStringsToChars(varargin{:});
    end

    % Use validatestring to ensure string and char inputs to 'operation'
    % are supported.
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

        jointOutput =  modelInfo.model_data(idxList).joints.joint_name;

        if nargout == 0
            % print list of model joint names
            printOutput =  modelInfo.model_data(idxList).model_name;
            robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                printOutput,'h','MODEL:');
            % print joint name list
            robotics.gazebo.internal.MATLABInterface.utils.printOutput(...
                jointOutput,'h','JOINTS:');
        else
            % returns joint name list
            varargout{1} = jointOutput;
        end

      case 'set'
        narginchk(5,inf);

        nargoutchk(0, 2);

        % validate joint name
        validateattributes(jointname, {'char'}, {'scalartext', 'nonempty'},....
                           'gzjoint', 'jointname');

        % validate and add name-value pair inputs
        parser = robotics.gazebo.internal.MATLABInterface.utils.validateInputs(...
            'SetJoint', varargin{:});
        % set model-joint parameter
        [status, errorMessage, printStatus, printMessage] = ...
            robotics.gazebo.internal.MATLABInterface.set.setJointParam(....
                modelname, jointname, parser, varargin{:});

        if nargout == 0
            robotics.gazebo.internal.MATLABInterface.utils.handleSetPrintOutput(...
                printStatus , printMessage);
        else
            varargout{1} = status;
            varargout{2} = errorMessage;
        end

      case 'get'
        narginchk(4,inf);

        validateattributes(jointname, {'char'}, {'scalartext', 'nonempty'},....
                           'gzjoint', 'jointname');

        % validate supported parameter
        supportedParameters = {'Position', 'Orientation','FudgeFactor',...
                            'CFM','SuspensionCFM','SuspensionERP','Axis0',...
                            'Axis1','Angle','XYZ','Damping','Friction'};

        validParamName = cell(1,numel(varargin));
        for vIdx = 1:numel(varargin)
            validParamName{vIdx} = validatestring(varargin{vIdx}, supportedParameters, 'gzjoint', 'parameter');
        end

        % get model-joint parameter
        [result, resultString] = ...
            robotics.gazebo.internal.MATLABInterface.get.getJointParam(...
                modelname, jointname, validParamName);

        if nargout == 0
            robotics.gazebo.internal.MATLABInterface.utils.handleGetPrintOutput(...
                modelname, jointname, resultString, validParamName, 'getJoint');
        else
            output = robotics.gazebo.internal.MATLABInterface.utils.handleGetReturnOutput(...
                result, validParamName, 'getJoint');

            nargoutchk(0, numel(output));

            varargout = cell(1,nargout);
            for outIdx = 1:nargout
                varargout{outIdx} = output{outIdx};
            end
        end
    end

end
