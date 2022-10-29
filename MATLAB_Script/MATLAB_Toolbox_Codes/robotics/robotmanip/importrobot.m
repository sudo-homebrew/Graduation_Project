function varargout = importrobot(input, varargin )
%IMPORTROBOT Import rigidBodyTree model from URDF and SDF file, text, or
%Simscape Multibody model
%   ROBOT = IMPORTROBOT(FILENAME) returns a rigidBodyTree object
%   by parsing the Unified Robot Description Format (URDF) robot
%   description or Simulation Description Format (SDF) model description
%   from file FILENAME.
%
%   ROBOT = IMPORTROBOT(TEXT) parses the URDF or SDF text. Specify TEXT as
%   a string scalar or character vector.
%
%   ROBOT = IMPORTROBOT(___, FORMAT) specifies the type of the robot
%   description file or text. The supported values of FORMAT are 'urdf' and
%   'sdf'.
%
%   [ROBOT] = IMPORTROBOT(___, Name, Value) provides additional options
%   specified by Name-Value pair arguments. Available parameter name:
%
%
%      'DataFormat'   - Data format for the RigidBodyTree object, specified
%                       as "struct", "row", or "column". To use dynamics
%                       methods, you must use either "row" or "column". The
%                       default data format is "struct".
%
%      'MeshPath'     - Search directories for mesh files,
%                       specified as a char vector or string,
%                       or a cell array of char vectors or strings.
%
%      'SDFModel'     - Select model name for SDF input containing multiple
%                       models.
%
%      'MaxNumBodies' - Maximum number of bodies allowed in imported robot
%                       during codegen and is effective only if greater than
%                       number of bodies specified in the input
%                       FILENAME/TEXT/MODEL.
%
%   [ROBOT, IMPORTINFO] = IMPORTROBOT(MODEL) returns a
%   rigidBodyTree object imported from the Simscape Multibody
%   components in the Simulink model specified by MODEL. MODEL may be the
%   name of a loaded model or model on the path, or the handle to a
%   Simulink model.
%
%   [ROBOT, IMPORTINFO] = IMPORTROBOT(___, Name, Value) provides additional
%   options specified by Name-Value pair arguments. Available parameter
%   names:
%
%      'BreakChains'      - Indicates whether to throw an error or break
%                           closed chains specified in the Simulink
%                           model. The imported rigid body tree has the
%                           chain closure joint removed.
%                           Options: 'error' (default) | 'remove-joints'
%
%      'ConvertJoints'    - Indicates whether to convert Simscape
%                           Multibody joints to fixed joints if there is
%                           no rigid body tree equivalent.
%                           Options: 'error' (default) | 'convert-to-fixed'
%
%      'DataFormat'       - Data format for the RigidBodyTree object,
%                           specified as "struct", "row", or "column". To
%                           use dynamics methods, you must use either
%                           "row" or "column". The default data format is
%                           "struct".
%
%      'SMConstraints'    - Indicates whether to remove constraint blocks
%                           from the imported rigid body tree.
%                           Options: 'error' (default) | 'remove'
%
%      'VariableInertias' - Indicates whether to remove variable inertia
%                           blocks from the imported rigid body tree.
%                           Options: 'error' (default) | 'remove'
%
%   Example 1 : Import robot from URDF file or string
%   -------------------------------------------------
%      % Import an LBR iiwa manipulator model from URDF file
%      lbr = importrobot('iiwa14.urdf')
%      show(lbr)
%
%      % It's also possible to specify a search directory for the mesh
%      % files indicated in URDF
%      rbt = importrobot('iiwa14.urdf', 'MeshPath', '..\meshes')
%      showdetails(rbt)
%
%      % Import from a minimalistic URDF string
%      s = '<?xml version="1.0" ?><robot name="min"><link name="L0"/></robot>';
%      mini = importrobot(s)
%
%   Example 2 : Import robot from SDF file or string
%   ------------------------------------------------
%      % Import an LBR iiwa manipulator model from SDF file
%      lbr = importrobot('iiwa14.sdf')
%      show(lbr)
%
%      % It's also possible to specify a search directory for the mesh
%      % files indicated in SDF
%      rbt = importrobot('iiwa14.sdf', 'MeshPath', '..\meshes')
%      showdetails(rbt)
%
%      % Import from a minimalistic SDF string
%      s = '<?xml version="1.0" ?><sdf version="1.6"><model name="min"><link name="L0"/></model></sdf>';
%      mini = importrobot(s)
%
%   Example 3 : Import robot from Simulink model
%   --------------------------------------------
%      % A simscape Multibody license is required to execute these
%      % functions and the following examples
%
%      % Open a simulink model containing Simscape Multibody
%      open_system('sm_import_humanoid_urdf.slx')
%
%      % Generate a rigidBodyTree and an import information
%      % object from the Simulink model
%      [robot, importInfo] = importrobot(gcs)
%
%      % See import details
%      showdetails(importInfo)
%
%   See also robotics.SMImporter, rigidBodyTreeImportInfo,
%            robotics.URDFImporter, robotics.SDFImporter

%   Copyright 2016-2021 The MathWorks, Inc.

%#codegen

    narginchk(1,9);

    if isempty(coder.target)
        %Determine type of first input
        [parsedInput, inputType] = processInputs(input, varargin{:});

        %Take different actions depending on the input type
        switch inputType
          case 'Text'
            % Get format for 'Text' input type
            fmt = lower(parsedInput.Results.format);
            switch(fmt)
              case 'urdf'
                % Add name-value pairs for URDF input
                param = {'MeshPath', parsedInput.Results.MeshPath,...
                         'SourceData', 'URDF'};
                importer = robotics.URDFImporter('DataFormat', parsedInput.Results.DataFormat, ...
                                                 'MaxNumBodies', parsedInput.Results.MaxNumBodies);
              case 'sdf'
                % Add name-value pairs for SDF input
                param = {'MeshPath', parsedInput.Results.MeshPath,...
                         'SourceData', 'SDF',...
                         'SDFModel',parsedInput.Results.SDFModel};
                importer = robotics.SDFImporter('DataFormat', parsedInput.Results.DataFormat, ...
                                                'MaxNumBodies', parsedInput.Results.MaxNumBodies);
            end

            robot = importer.importrobot(parsedInput.Results.input, param{:});

            %Assign outputs
            varargout{1} = robot;

          case 'Simulink'
            %If the file exist on the path but is not loaded, load it into
            %memory and get the handle. If it is already loaded, remove the
            %extension so only the model name is passed to the importer.
            [~, fileBody] = fileparts(parsedInput.Results.input);
            if ~bdIsLoaded(fileBody)
                sys = load_system(parsedInput.Results.input);
            else
                sys = get_param(fileBody, 'Handle');
            end

            %Create importer object
            importer = robotics.SMImporter('BreakChains', parsedInput.Results.BreakChains, ...
                                           'ConvertJoints', parsedInput.Results.ConvertJoints, ...
                                           'SMConstraints', parsedInput.Results.SMConstraints, ...
                                           'VariableInertias', parsedInput.Results.VariableInertias, ...
                                           'DataFormat', parsedInput.Results.DataFormat, ...
                                           'MaxNumBodies', parsedInput.Results.MaxNumBodies);

            %Import robots and assign outputs
            [robot, importInfo] = importer.importrobot(sys);
            varargout{1} = robot;
            varargout{2} = importInfo;
        end
    else
        %For coder, make an extrinsic call to internal extractStructFromRBT that returns a
        %codegen compatible RBT struct
        rbt = coder.const(@feval, 'robotics.manip.internal.populateRBTStructFromFunction', ...
                          'importrobot', input, varargin{:});

        %Convert RBT struct back to rigidBodyTree object
        robot = rigidBodyTree(robotics.manip.internal.RigidBodyTree(rbt.MaxNumBodies, rbt));

        varargout{1} = robot;
    end


    function [p, idType] = processInputs(argIn, varargin)
        processedInput = convertStringsToChars(argIn);

        %Verify that first input is nonempty
        validateattributes(processedInput, {'char','string','double'},{'nonempty'},'importrobot', '', 1);

        if isnumeric(argIn)
            try %#ok<*EMTC>
                processedInput = get_param(processedInput, 'Name');
            catch
                error(message('robotics:robotmanip:urdfimporter:NumericInputs'));
            end
        end

        %Initialize the output to empty
        idType = [];
        %Initialize the source type to empty
        sourceType = [];

        %Determine the type of conversion based on the main input. To avoid
        %calling Simulink unless it is necessary, try other routes first.
        %There are four valid options:
        %   1) URDF or SDF file (with appropriate extension)
        %   2) Text from a URDF or SDF
        %   3) Simulink model name
        %   4) Name of a subsystem within a larger Simulink model

        %First check if the input is a file with URDF or SDF extension
        [~,~,fileExtension] = fileparts(processedInput);

        % Check 'format' is specified for 'file' or 'text' input
        if nargin > 1
            if strcmpi(varargin{1}, 'sdf')
                % if input FORMAT is 'sdf' then file extension should be
                % any except '.urdf'
                if strcmpi(fileExtension, '.urdf')
                    error(message('robotics:robotmanip:urdfimporter:NoMatchingFormat','urdf'));
                else
                    sourceType = 'SDF';
                end
            elseif strcmpi(varargin{1}, 'urdf')
                % if input FORMAT is 'urdf' then file extension should be
                % any except '.sdf'
                if strcmpi(fileExtension, '.sdf')
                    error(message('robotics:robotmanip:urdfimporter:NoMatchingFormat','sdf'));
                else
                    sourceType = 'URDF';
                end
            end
        end

        % If 'format' is not specified then select 'idType' as per input
        if isempty(sourceType)
            if strcmpi(fileExtension, '.urdf')
                %Identify URDF files by their file extension
                sourceType = 'URDF';
            elseif strcmpi(fileExtension, '.sdf')
                %Identify SDF files by their file extension
                sourceType = 'SDF';
            elseif ~isempty(regexpi(processedInput, '<robot') & regexpi(processedInput, '</robot>') )
                %Identify URDFText, which will include robot declaration tags
                sourceType = 'URDF';
            elseif ~isempty(regexpi(processedInput, '<sdf') & regexpi(processedInput, '</sdf>') )
                %Identify SDFText, which will include sdf declaration tags
                sourceType = 'SDF';
            elseif (exist(processedInput, 'file') == 4) || strcmpi(fileExtension, '.mdl') || strcmpi(fileExtension, '.slx')
                %Identify simulink models by file extensions if possible.
                sourceType = 'Simulink';
            else
                %If the string refers to the subsystem of a model, then it must
                %be redirected to the root model in order to be found as a
                %Simulink model
                try %#ok<*EMTC>
                    processedInput = bdroot(processedInput);
                    if exist(processedInput, 'file') == 4
                        sourceType = 'Simulink';
                    end
                catch cause_ME
                    %If none of the above options can be validated, the input
                    %is invalid.
                    base_ME = MException(message('robotics:robotmanip:urdfimporter:InvalidFirstInput'));
                    new_ME = addCause(base_ME, cause_ME);
                    throw(new_ME);
                end
            end
        end

        %Set up input parser based on input type
        p = inputParser;
        switch sourceType
          case 'Simulink'
            addRequired(p,'input');
            addParameter(p, 'BreakChains', 'error');
            addParameter(p, 'ConvertJoints', 'error');
            addParameter(p, 'SMConstraints', 'error');
            addParameter(p, 'VariableInertias', 'error');
            addParameter(p, 'DataFormat', 'struct');
            addParameter(p, 'MaxNumBodies', 0);

            idType = 'Simulink';
          case 'URDF'
            addRequired(p,'input');
            addOptional(p,'format', 'urdf', @(x) ischar(validatestring(x, {'urdf'}, 'importrobot', 'format')) );
            addParameter(p,'MeshPath', {});
            addParameter(p, 'DataFormat', 'struct');
            addParameter(p, 'MaxNumBodies', 0);

            if nargin > 1
                % Meshpath can be a cell array of character and/or string inputs
                if nargin > 2 && isa(varargin{end},'cell')
                    cellData = varargin{end};
                    varargin{end} = convertStringsToChars(cellData{:});
                end

                [varargin{:}] = convertStringsToChars(varargin{:});
            end
            idType = 'Text';

          case 'SDF'
            addRequired(p,'input');
            addOptional(p,'format', 'sdf', @(x) ischar(validatestring(x, {'sdf'}, 'importrobot', 'format')) );
            addParameter(p,'MeshPath', {});
            addParameter(p,'SDFModel','', @(x) validateattributes(x,{'char','string'}, {'nonempty'},'importrobot', 'SDFModel') );
            addParameter(p, 'DataFormat', 'struct');
            addParameter(p, 'MaxNumBodies', 0);

            % Meshpath can be a cell array of character and/or string inputs
            % get MeshPath index
            meshIdx = find(strcmp(varargin,'MeshPath') == 1);
            if isempty(meshIdx)
                meshIdx = 1;
            end
            if nargin > 2 && nargin > (meshIdx+1)
                %  mesh should be provided as input, so required to
                %  check meshIdx+1 exist
                if contains('MeshPath',varargin{meshIdx}) &&...
                        isa(varargin{meshIdx+1},'cell')
                    cellData = varargin{meshIdx+1};
                    varargin{meshIdx+1} = convertStringsToChars(cellData{:});
                end
            end
            [varargin{:}] = convertStringsToChars(varargin{:});
            idType = 'Text';
        end

        %Parse inputs
        parse(p, processedInput, varargin{:});
    end
end
