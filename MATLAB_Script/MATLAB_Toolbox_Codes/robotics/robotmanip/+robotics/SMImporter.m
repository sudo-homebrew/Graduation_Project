classdef SMImporter < robotics.manip.internal.Importer ...
        & matlab.mixin.Copyable
    %SMIMPORTER Create a Simscape Multibody Importer object.
    %
    %   IMPORTER = robotics.SMIMPORTER() returns a Simscape Multibody
    %   importer object IMPORTER with default property settings
    %
    %   IMPORTER = robotics.SMIMPORTER('PropertyName', PropertyValue, ...)
    %   sets each specified property of the Simscape Multibody importer
    %   object IMPORTER to the specified value upon construction.
    %
    %   SMIMPORTER properties
    %      BreakChains      - Indicates whether to throw an error or remove and break closed Simscape Multibody chains in the imported rigid body tree
    %      ConvertJoints    - Indicates whether to convert Simscape Multibody joints to fixed joints if there is no rigid body tree equivalent.
    %      SMConstraints    - Indicates whether to remove constraint blocks from the imported rigid body tree
    %      VariableInertias - Indicates whether to remove variable inertia blocks from the imported rigid body tree
    %      ParseInertial    - Indicates whether to parse inertial parameters
    %
    %   SMIMPORTER methods
    %      importrobot     - Import rigidBodyTree models from a Simulink model containing Simscape Multibody systems
    %
    %   Example
    %      % A simscape Multibody license is required to execute these
    %      % functions and the following examples
    %
    %      % Open a simulink model containing Simscape Multibody
    %      sm_four_bar
    %
    %      % Create a Simscape Multibody importer
    %      importer = robotics.SMImporter();
    %
    %      % Break closed chains in the source model
    %      importer.BreakChains = 'remove-joints'
    %
    %      % Convert source model joints to fixed when no RST equivalent exists
    %      importer.ConvertJoints = 'convert-to-fixed'
    %
    %      % Generate a rigidBodyTree and an import information
    %      % object from the Simulink model
    %      [robot, importInfo] = importrobot(importer, gcs);
    %
    %      % See import details
    %      showdetails(importInfo)
    %
    %   See also importrobot

    %   Copyright 2018-2021 The MathWorks, Inc.
    %

    properties
        %BreakChains Indicates whether to break closed chains in Simscape Multibody model
        %   Simscape Multibody supports closed chains, which must be broken
        %   in order to be converted to rigidBodyTree. This
        %   parameter can be set to 'error' to throw an error when closed
        %   chains are present, or to 'remove-joints' to break the chains
        %   by removing a chain closure joint, which will produce a
        %   warning.
        %
        %   Default: 'error'
        BreakChains

        %ConvertJoints Indicates whether to convert Simscape Multibody joints without RST equivalents
        %   The SMImporter can import Revolute, Prismatic, and Weld joints
        %   from a Simscape Multibody model. All other joints are
        %   unsupported. This parameter can be set to 'error' to error out
        %   when other joints are present, or 'convert-to-fixed' to convert
        %   other joints to fixed joints when importing, which will produce
        %   a warning.
        %
        %   Default: 'error'
        ConvertJoints

        %SMConstraints Indicates whether to remove Simscape Constraints from imported tree
        %   This parameter can be set to 'error' to error out when a source
        %   model contains constraint blocks, which are unsupported in
        %   rigidBodyTree, or 'remove' to remove these block
        %   contributions from the imported tree, which will produce a
        %   warning.
        %
        %   Default: 'error'
        SMConstraints

        %VariableInertias Indicates whether to remove variable inertias from imported tree
        %   This parameter can be set to 'error' to error out when a source
        %   model contains blocks with variable mass or inertia, which are
        %   unsupported in rigidBodyTree, or 'remove' to remove
        %   these block contributions from the imported tree, which will
        %   produce a warning.
        %
        %   Default: 'error'
        VariableInertias

        %ParseInertial Indicates whether to parse inertial parameters
        %   This is a Boolean property. Inertial parameters include body
        %   mass, center of mass and inertia.
        %   tensor.
        %
        %   Default: true
        ParseInertial
        
        %DataFormat Defines data format for the RigidBodyTree object
        %   DataFormat is specified as "struct", "row", or "column". To use
        %   dynamics methods, you must use either "row" or "column".
        %
        %   Default: 'struct'
        DataFormat

        %MaxNumBodies Maximum number of bodies on the RigidBodyTree object
        %   This property is only required during codegen.
        %   
        %   Default: 0
        MaxNumBodies = 0
    end

    properties (Constant, Access=?robotics.manip.internal.InternalAccess)

        %SupportedSMJointDetails List of supported SM joints and
        %corresponding RBT joint
        SupportedSMJointDetails = {{'Revolute Joint', 'revolute'}, ...
                            {'Prismatic Joint', 'prismatic'}, ...
                            {'Weld Joint', 'fixed'}}
    end

    methods
        function obj = SMImporter(varargin)
        %SMImporter Constructor

            parser = inputParser;
            parser.addParameter('BreakChains', 'error');
            parser.addParameter('ConvertJoints', 'error');
            parser.addParameter('SMConstraints', 'error');
            parser.addParameter('VariableInertias', 'error');
            parser.addParameter('ParseInertial', true);
            parser.addParameter('DataFormat', 'struct');
            parser.addParameter('MaxNumBodies', 0);
            parser.parse(varargin{:});

            obj.ParseInertial = parser.Results.ParseInertial;
            obj.BreakChains = parser.Results.BreakChains;
            obj.ConvertJoints = parser.Results.ConvertJoints;
            obj.SMConstraints = parser.Results.SMConstraints;
            obj.VariableInertias = parser.Results.VariableInertias;
            obj.DataFormat = parser.Results.DataFormat;
            obj.MaxNumBodies = parser.Results.MaxNumBodies;
        end

        function [robots, importInfo] = importrobot(obj, sys)
        %importrobot Generate rigidBodyTree objects from Simscape Multibody systems in Simulink models
        %   This method generates an urdfshared.Model object (or
        %   cell array of urdfshared.Model objects) from a compatible
        %   loaded Simulink system, and then imports that to
        %   rigidBodyTree. After initial import, the bodies
        %   are renamed so that the base is called "Base", and the
        %   other bodies are named Body1...BodyN. Here, sys is a handle
        %   to a Simulink model that is open or loaded in memory.

        % This method requires Simscape Multibody. Display an error if
        % no license is available
            if ~robotics.internal.license.isSimscapeMultibodyLicensed
                error(message('shared_robotics:license:NoMechLicense','importrobot'));
            end

            % Call multibody utility to export SM system to
            % urdfshared.Model or a cell array of urdfshared.Model objects.
            % The utility updates the model and extracts the
            % sm.system, then converts that to a sharedModel. If there are
            % closed chains, they are broken (and the spanning tree is
            % returned). Additionally, all joint information is parsed, but
            % must be converted to RBT-compatible form, as done below.
            [sharedModels, translationData] = simscape.multibody.sli.internal.toURDF(sys);

            % Handle import errors
            modelName = get_param(sys,'Name');
            if isempty(sharedModels)
                obj.processTranslationIssues(modelName, translationData);
            end

            % Get a list of all the blocks in the Simulink model
            allModelBlocks = find_system(modelName, 'LookUnderMasks','on');

            numModels = length(sharedModels);
            robots = cell(numModels,1);
            importInfo = cell(numModels,1);

            for i = 1:numModels

                model = sharedModels(i);
                data = translationData(i);

                %React to import data (for each robot) and produce a single
                %structure that is specific to this shared model
                modelImportData = obj.processTranslationFlags(data);

                % Process joints so that they are compatible with RBT
                [jointNameMap, convertedJts, convertedJtTypes] = obj.processSMJoints(model.Joints, data.JointMap);

                % Add conversion info to translationData struct
                modelImportData.ConvertedJoints = convertedJts;
                modelImportData.ConvertedJointTypes = convertedJtTypes;

                % Need to update later for multiple SM systems / robots
                robot = obj.importURDFShared(model);

                %Set robot DataFormat
                robot.DataFormat = obj.DataFormat;
                
                % Rename robot body and joint names
                [bodyMap, jointMap] = obj.processRBTNames(robot, modelImportData.LinkComponents, jointNameMap);

                % Create traceability output
                importInfo{i} = rigidBodyTreeImportInfo(robot, modelName, ...
                                                                 bodyMap, jointMap, modelImportData, allModelBlocks);
                
                % Assign into cell array
                robots{i} = robot;
            end

            if numModels == 1
                robots = robots{:};
                importInfo = importInfo{:};
            end
        end
    end

    methods (Access = ?robotics.manip.internal.InternalAccess)
        function processTranslationIssues(~, sysName, data)
        %processTranslationIssues Notify user of any model-level conversion errors

        % Throw the correct exception of there is a compilation error
            if isfield(data, 'HasCompilationError') && data.HasCompilationError
                %Combine the original Simulink compilation error with a
                %MATLAB message that tells the user that the problem is
                %with the model in Simulink (as a cause in the exception)
                slExc = data.CompilationError;
                mlExc = MException('robotics:robotmanip:urdfimporter:CompilationError', ...
                                   message('robotics:robotmanip:urdfimporter:CompilationError',sysName).getString);
                userExc = addCause(slExc, mlExc);

                throwAsCaller(userExc);
            end

            % Indicate to user if there is no SM system within this model
            if ~data.HasMultibody
                error(message('robotics:robotmanip:urdfimporter:NoMultibodyModel', sysName));
            end
        end

        function modData = processTranslationFlags(obj, data)
        %processTranslationFlags Notify user of any conversion issues
        %   The toURDF import utility provides a number of flags in the
        %   translationData output that can be used to indicate when
        %   the import conversion is not one-to-one. This function
        %   turns those flags into warnings/errors that are provided to
        %   the user.

        % Error if the system contains flexible bodies
            if ~isempty(data.FlexibleBodies)
                flexBodyList = strjoin(data.FlexibleBodies, '\n');
                error(message('robotics:robotmanip:urdfimporter:FlexibleBodiesError',flexBodyList));
            end

            % Notify user if SM system has constraints that must be ignored
            if ~isempty(data.Constraints)
                constraintList = strjoin(data.Constraints, '\n');
                if strcmp(obj.SMConstraints, 'error')
                    error(message('robotics:robotmanip:urdfimporter:ConstraintError',constraintList));
                else
                    warning(message('robotics:robotmanip:urdfimporter:ConstraintWarning',constraintList));
                    if ~data.IsTree && isempty(data.CutJoints)
                        %If the source is not a tree, but there are no cut
                        %joints, the tree was actually created by
                        %constraints. Ignoring the constraints will open
                        %the tree.
                        data.IsTree = true;
                    end
                end
            end

            % Notify user if SM includes variable inertias that must be ignored
            if ~isempty(data.VariableInertias)
                varInertiaList = strjoin(data.VariableInertias, '\n');
                if strcmp(obj.VariableInertias, 'error')
                    error(message('robotics:robotmanip:urdfimporter:VarInertiaError',varInertiaList));
                else
                    warning(message('robotics:robotmanip:urdfimporter:VarInertiaWarning',varInertiaList));
                end
            end

            % Check if the SM system is a tree
            if (~data.IsTree)
                cutJointsList = strjoin(data.CutJoints, '\n');

                % Notify user based on 'BreakChains' N-V pair setting
                if strcmp(obj.BreakChains, 'error')
                    error(message('robotics:robotmanip:urdfimporter:ClosedChainError', cutJointsList));
                else
                    warning(message('robotics:robotmanip:urdfimporter:ClosedChainWarning', cutJointsList));
                end
            end

            % Notify user if SM system produces implicit joints
            if ~isempty(data.ImplicitJoints)
                implJointsList = strjoin(data.ImplicitJoints, '\n');
                warning(message('robotics:robotmanip:urdfimporter:ImplicitJoints',implJointsList));
            end

            modData = data;
        end

        function [jointNameMap, convertedJointsPath, convertedJointsType] = processSMJoints(obj, jointMap, oldJointNameMap)
        % processSMJoints Modify the joints container in the shared model for compatibility
        %   By default, Simscape Multibody provides an urdfShared model
        %   that contains all joints, including ones that are not supported
        %   by RBT. This function converts unsupported joints to fixed
        %   joints.  When urdfshared.Model objects are generated by
        %   Simscape Multibody, the axes are left unassigned unless the
        %   joint is a prismatic or revolute joint (there is actually a
        %   private property in the urdfshared.Joint called isURDF, which,
        %   when false, allows the urdfshared axis to be left empty or
        %   given an arbitrary assignment). To ensure positive displacement
        %   of prismatic and revolute joints is consistent in Simscape
        %   Multibody and the RBT, if the base port of the Multibody joint
        %   is connected to its parent link (i.e., towards the root of the
        %   kinematic tree) and the follower port is connected to its child
        %   link, then the joint axis in the RBT is set to the positive
        %   z-direction; if these connections are switched, then the joint
        %   axis in the RBT is set to the negative z-direction.

        %Converted Joint Information
            convertedJointsCount = 0;
            convertedJointsPath = {};
            convertedJointsType = {};

            jointKeys = jointMap.keys;
            jointNameMap = containers.Map.empty;

            for i = 1:jointMap.length
                joint = jointMap(jointKeys{i});

                %Check if the joint is among the supported types and get
                %the index if so
                supportedJointList = cellfun(@(x)x(1),obj.SupportedSMJointDetails);
                typeIndex = find(strcmp(supportedJointList, joint.Type),1);

                if ~isempty(typeIndex)
                    %Apply the details using the joint details property
                    details = obj.SupportedSMJointDetails{typeIndex};
                    joint.Type = details{2};
                else
                    % Convert all other SM joints to fixed joints and store
                    % data so that an appropriate error or warning can be
                    % thrown after all problem joints are recognized.
                    convertedJointsCount = convertedJointsCount + 1;
                    convertedJointsType{convertedJointsCount} = joint.Type; %#ok<AGROW>

                    % Fill in names for block paths
                    convertedJointsPath{convertedJointsCount} = jointKeys{i}; %#ok<AGROW>

                    %Convert the joint to fixed
                    joint.Type = 'fixed';
                    % Leave joint axis empty (cannot set to empty)
                end

                %Fill in joint name map using data from the translationData
                %joint name map.
                jointNameMap(jointKeys{i}) = oldJointNameMap(jointKeys{i});
            end

            %Throw the correct error or warning depending on user input
            if convertedJointsCount > 0
                convertedJointPathsList = strjoin(convertedJointsPath,'\n');
                if strcmp(obj.ConvertJoints, 'convert-to-fixed')
                    warning(message('robotics:robotmanip:urdfimporter:JointImportWarning', convertedJointPathsList));
                else
                    error(message('robotics:robotmanip:urdfimporter:JointImportError', convertedJointPathsList));
                end
            end
        end

        function [bodyNameMap, jointNameMap] = processRBTNames(~, robot, urdfLinkMap, urdfJointMap)
        %processRBTNames Rename rigid bodies used set nomenclature
        %   This function uses the replaceBody method to rename all the
        %   bodies in a rigidBodyTree object with [Base Body1
        %   Body2 ... BodyN] syntax. The function outputs a
        %   containers.Map object that takes these new body names as
        %   keys and outputs the block paths as values. This
        %   essentially replaces the urdfLinkMap input that does the
        %   same, but using the original Link names as keys.

        % Map name changes for traceability
            bodyNameMap = containers.Map.empty;
            jointNameMap = containers.Map.empty;

            % Change base name first
            bodyNameMap('Base') = urdfLinkMap(robot.BaseName);
            robot.BaseName = 'Base';

            %Determine the number of digits in the numbering scheme. The
            %aim is to avoid having any bodies that share substrings, so
            %that traceability tools do not return two outputs when the
            %search is only for one.
            numDigits = floor(log10(robot.NumBodies)) + 1;

            % Change remaining body names via replaceBody function
            for i=1:robot.NumBodies

                newBody = copy(robot.Bodies{i});
                newBody.Name = sprintf('Body%0*i', numDigits, i);

                %Transfer joint information
                oldJointName = robot.Bodies{i}.Joint.Name;
                newBody.Joint.Name = sprintf('Joint%0*i', numDigits, i);
                jointNameMap(newBody.Joint.Name) = urdfJointMap(oldJointName);

                %Record name changes and also remap body contents to a new
                %map.
                bodyNameMap(newBody.Name) = urdfLinkMap(robot.Bodies{i}.Name);

                %Replace body to make change
                robot.replaceBody(robot.Bodies{i}.Name, newBody);
            end
        end
    end

    methods
        function set.ParseInertial(obj, b)
        %set.ParseInertial
            validateattributes(b, {'logical', 'numeric'}, {'nonempty','scalar'}, ...
                               'URDFImporter','ParseInertial');
            obj.ParseInertial = logical(b);
        end

        function set.BreakChains(obj, s)
        %set.BreakChains
            validStrings = {'error', 'remove-joints'};
            obj.BreakChains = validatestring(s, validStrings, 'robotics.SMImporter', 'obj.BreakChains');
        end

        function set.ConvertJoints(obj, s)
        %set.ConvertJoints
            validStrings = {'error', 'convert-to-fixed'};
            obj.ConvertJoints = validatestring(s, validStrings, 'robotics.SMImporter', 'obj.ConvertJoints');
        end

        function set.SMConstraints(obj, s)
        %set.SMConstraints
            validStrings = {'error', 'remove'};
            obj.SMConstraints = validatestring(s, validStrings, 'robotics.SMImporter', 'obj.SMConstraints');
        end

        function set.VariableInertias(obj, s)
        %set.VariableInertias
            validStrings = {'error', 'remove'};
            obj.VariableInertias = validatestring(s, validStrings, 'robotics.SMImporter', 'obj.VariableInertias');
        end
        
        function set.DataFormat(obj, dataformat)
        %set.DataFormat
            validateattributes(dataformat,{'char','string'},{'nonempty','scalartext'},'robotics.SMImporter','DataFormat');
            obj.DataFormat = validatestring(dataformat, {'struct', 'row', 'column'}, 'robotics.SMImporter', 'DataFormat');
        end
    end
end
