classdef analyticalInverseKinematics < robotics.manip.internal.InternalAccess
    %analyticalInverseKinematics Create an analytical inverse kinematics solver
    %   This object is used to generate analytical solvers for a supported
    %   subset of 6-DoF rigid body trees. The object uses a user-defined
    %   kinematic group to define a tree for which to solve the inverse
    %   kinematics problem, and generates solutions as standalone
    %   functions. Kinematic groups are validated by default, but to see
    %   all possible supported kinematic groups for a robot for which
    %   inverse kinematics can be computed, use the showdetails method. For
    %   other robots, please consider a numerical solver.
    %
    %   AIK = analyticalInverseKinematics(tree) creates an analytical
    %   inverse kinematics solver for the tree using the last body in the
    %   rigidBodyTree.Bodies array as the end effector, computed relative
    %   to the base.
    %
    %   AIK = analyticalInverseKinematics(___, 'PropertyName', PropertyValue, ..)
    %   sets additional properties specified as name-value pairs. You can
    %   specify multiple properties in any order.
    %
    %   ANALYTICALINVERSEKINEMATICS Properties:
    %      KinematicGroup      - Struct that defines the sub-chain for which inverse kinematics will be generated
    %
    %   Properties that can only be user-defined at construction:
    %      RigidBodyTree          - Rigid body tree robot model
    %
    %   Read-only properties:
    %      KinematicGroupType        - Kinematic group classification type
    %      KinematicGroupConfigIdx   - Mapping of IK solution configuration to rigid body tree configuration
    %      IsValidGroupForIK         - Indicates whether a closed-form solution can be generated
    %
    %   ANALYTICALINVERSEKINEMATICS object functions:
    %      showdetails          - Show overview of compatible kinematic groups
    %      generateIKFunction   - Generate a MATLAB function to compute inverse kinematics in closed-form
    %
    %   Reference:
    %   Pieper, D. The Kinematics Of Manipulators Under Computer Control.
    %   Stanford University (1968).
    %
    %   Example:
    %      % Load a robot. Use 'row 'format to work most easily with the
    %      % analytical inverse kinematics output.
    %      robot = loadrobot('abbIrb120', 'DataFormat', 'row');
    %
    %      % Create an analytical inverse kinematics object &
    %      % generate an IK function.
    %      ikObj = analyticalInverseKinematics(robot);
    %      ikFcn = generateIKFunction(ikObj, 'ikAbbIRB120');
    %
    %      % Create a feasible target pose and solve for it using
    %      % inverse kinematics.
    %      tgtPose = trvec2tform([0 0.25 0.25]);
    %      tgtConfig = ikFcn(tgtPose);
    %
    %      % Visualize result.
    %      show(robot, tgtConfig(1,:));
    %      hold all;
    %      plotTransforms(tform2trvec(tgtPose), tform2quat(tgtPose));
    %
    %   See also: inverseKinematics, generalizedInverseKinematics
    
    %   Copyright 2020-2021 The MathWorks, Inc.
    
    properties (Dependent)
        %KinematicGroup - Struct that defines the sub-chain for which inverse kinematics will be generated
        %   The kinematic group is a structure that defines a kinematic
        %   chain for which an inverse kinematics solution can be found.
        %   The structure has four fields:
        %      BaseName               - The body name in the associated
        %                               rigid body tree that corresponds to
        %                               the base of the chain. The default
        %                               value is the base of the associated
        %                               rigid body tree.
        %
        %      EndEffectorBodyName    - The body name in the associated
        %                               rigid body tree that corresponds to
        %                               the end-effector of the chain. The
        %                               default value is the last body in
        %                               the associated rigid body tree.
        %
        %   When no Kinematic group is provided in the constructor, a
        %   structure with default fields is used.
        KinematicGroup
    end
    
    properties (SetAccess = private)
        
        %RigidBodyTree - Rigid body tree robot model
        %   Rigid body tree robot model, specified as a rigidBodyTree
        %   object that defines the inertial and kinematic properties of
        %   the manipulator. The RigidBodyTree property is defined at
        %   object construction and is read-only.
        RigidBodyTree
    end
    
    properties (SetAccess = private, Dependent)
        %KinematicGroupType - Kinematic group classification type
        %   The type of the kinematic group, as identified during
        %   validation. The kinematic group classification type is an
        %   character of the form X1-X2-X3-...-XN for the N number of non-fixed joints in
        %   the rigid body tree defined by the kinematic group. Each value
        %   of Xi can be either:
        %      R   - Indicates the joint in that position is revolute
        %      P   - Indicates the joint in that position is prismatic
        %      S   - Indicates the joint in that position is revolute but
        %            is part of a spherical joint created by three
        %            consecutive intersecting revolute joints.
        %   The KinematicGroupType property is updated when KinematicGroup
        %   is set and is read-only.
        KinematicGroupType
        
        %KinematicGroupConfigIdx - Mapping of IK solution configuration to rigid body tree configuration
        %   This 6-element vector contains the indices of the source
        %   rigidBodyTree's configuration that are changed by the outcome
        %   of the IK solution.
        KinematicGroupConfigIdx
        
        %IsValidGroupForIK - Indicates whether a closed-form solution can be generated
        %   This is a boolean value that is true when a closed-form inverse
        %   kinematics solution can be found for the associated kinematic
        %   group, and false otherwise. This property is updated when
        %   KinematicGroup is set and is read-only.
        IsValidGroupForIK
    end
    
    properties (SetAccess = private, GetAccess = ?robotics.manip.internal.InternalAccess)
        
        %TreeNumDoF Number of degrees of freedom in the RigidBodyTree
        TreeNumDoF
        
        %SubChainNumDoF Number of degrees of freedom in the tree defined by the kinematic group
        SubChainNumDoF
        
        %SubChain Rigid body tree defined by the kinematic group
        SubChain
        
        %SubchainBodyIdMap Vector that maps the subchain base and bodies to the original tree
        %   This property corresponds to the IDs of the bodies in the
        %   original rigidBodyTree that are in the subchain.
        SubchainBodyIdMap
        
        %KinematicGroupInternal Stored version of KinematicGroup
        KinematicGroupInternal
        
        %IdentificationTol Tolerance used in the identification step
        IdentificationTol = sqrt(eps)
        
        %SolutionTol Tolerance used during solution for equality checks
        SolutionTol = 1e-6;
        
        %SolutionDetails Structure containing overview of solution approach
        SolutionDetails = struct()
        
        %DetailsData Struct containing the data used by the showdetails method
        %   Since the showdetails method always produces the same output,
        %   it stores the associated data to minimize the amount of times
        %   the data needs to be generated. This struct stores that
        %   content. It has three fields:
        %
        %      DisplayText       String containing output of showdetails
        %
        %      LastLinkObjName   The last object name used in the links. If
        %                        this value does not match the current name
        %                        of the associated object, the links need
        %                        to be regenerated (this is checked each
        %                        time showdetails is called).
        %
        %      CalcDetails       Struct containing the analysis results
        %                        from the showdetails assessment, i.e. the
        %                        all compatible kinematic groups
        DetailsData = struct('DisplayText', [], 'LastLinkObjName', [], 'CalcDetails', [])
    end
    
    methods
        function obj = analyticalInverseKinematics(tree, varargin)
            %AnalyticalIK Object constructor
            
            % Input validation & processing
            narginchk(1, 3);
            obj.initializeTreeProperties(tree);
            
            % Process options specified in constructor
            charInputs = cell(1,nargin-1);
            [charInputs{:}] = convertStringsToChars(varargin{:});
            names = {...
                'KinematicGroup', ...
                };
            defaults = {...
                obj.getDefaultKinematicGroup, ...
                };
            
            % Parse inputs
            parser = robotics.core.internal.NameValueParser(names, defaults);
            parse(parser, charInputs{:});
            
            % Validate kinematic group
            obj.validateAndSetKinematicGroup(parameterValue(parser, 'KinematicGroup'));
            
            % Classify structure & determine feasibility for IK
            obj.SolutionDetails = obj.assessKinematicGroupForIK(obj.SubChain, obj.SubChainNumDoF);
        end
        
        function kinGroupDetails = showdetails(obj)
            %showdetails Show overview of compatible kinematic groups
            %   kinGroupDetails = showdetails(analyticalIK) displays the
            %   details of all kinematic group combinations in the
            %   associated rigid body tree object for which inverse
            %   kinematics solutions can be found. The method outputs a
            %   P-element structs array containing the calculation details
            %   of the analysis, were P is the number of possible 6-DoF
            %   robots that can be formed from the rigidBodyTree. The first
            %   Q elements correspond to the Q indices that produce valid
            %   analytical IK solutions and appear in the display output.
            %
            %   The structure has the following fields:
            %      KinematicGroup                 - A struct specifying the
            %                                       end effector and base
            %                                       name
            %
            %      Type                           - Kinematic group
            %                                       classification type,
            %                                       matching the syntax of
            %                                       the KinematicGroupType
            %                                       property
            %
            %      IsIntersectingAxesMidpoint     - A 1xN vector indicating
            %                                       whether the ith joint
            %                                       axis intersects with
            %                                       the joint axes of the
            %                                       preceding and following
            %                                       moving joints at a
            %                                       point
            %
            %      MidpointAxisIntersections      - A 2x3xN matrix that
            %                                       stores the joint
            %                                       intersection points
            %                                       with respect to the
            %                                       middle frame. For the
            %                                       ith joint (specified in
            %                                       the 3rd dimension),
            %                                       this property returns a
            %                                       2x3 matrix where the
            %                                       first row is the
            %                                       intersection of the
            %                                       previous axis with the
            %                                       current joint axis, and
            %                                       the second row is the
            %                                       intersection of the
            %                                       current joint axis with
            %                                       the next joint axis.
            %                                       All intersections are
            %                                       in the ith joint frame.
            %                                       Axes that don't
            %                                       intersect return NaNs.
            %
            %   Example:
            %       % Load a robot kinova Jaco, which has 15 total bodies
            %       robot = loadrobot('kinovaJacoJ2S6S300');
            %
            %       % Create an analytical inverse kinematics object with
            %       % default kinematic group parameters
            %       aik = analyticalInverseKinematics(robot);
            %
            %       % See all possible kinematic group configurations
            %       info = showdetails(aik);
            %
            %       % Click "Apply configurations" to apply the kinematic
            %       % group to the analytical inverse kinematics object, or
            %       % use the programmatic API. For example, the following
            %       % command sets the kinematic group to the fourth listed
            %       % value in the showdetails table:
            %       aik.KinematicGroup = info(4).KinematicGroup;
            %
            %       % Generate code
            %       generateIKFunction(aik, 'robotIK');
            
            % Get the object name for use in showdetails links
            objName = inputname(1);
            
            % Since the underlying rigidBodyTree doesn't change, the
            % compatibility analysis output by showdetails only needs to be
            % run once
            if isempty(obj.DetailsData.CalcDetails)
                % Determine all possible kinematic groups that are valid
                % for IK
                obj.DetailsData.CalcDetails = obj.getAllValidKinematicGroups;
            end
            
            % The text needs to be regenerated when it doesn't exist, or
            % when the object's name has changed (the name is used in the
            % links to apply configurations to the kinematic groups).
            if isempty(obj.DetailsData.DisplayText) || ~strcmp(objName, obj.DetailsData.LastLinkObjName)
                
                % Update the stored object name
                obj.DetailsData.LastLinkObjName = objName;
                
                % If the kinematic group analysis returned nothing, the
                % showdetails method just returns a string indicating that
                % to the user
                if isempty(obj.DetailsData.CalcDetails)
                    obj.initializeShowDetailsOutput();
                    obj.DetailsData.DisplayText = [obj.DetailsData.DisplayText message('robotics:robotmanip:analyticalinversekinematics:NoValidKinematicGroupsFound').getString];
                else
                    % If the kinematic group returned values, they must be
                    % translated to a tabular text format
                    
                    % Initialize output text
                    displayWidths = obj.initializeShowDetailsOutput();
                    
                    % Update showdetails object by appending display lines
                    % for each compatible kinematic group in the
                    % calculation details struct
                    for i = 1:numel(obj.DetailsData.CalcDetails)
                        obj.addDisplayDetailsLine(displayWidths, objName, i, obj.DetailsData.CalcDetails(i));
                    end
                end
            end
            
            % Output details to the screen
            fprintf('%s\n',obj.DetailsData.DisplayText);
            
            % Return an output only if queried
            if nargout > 0
                kinGroupDetails = obj.DetailsData.CalcDetails;
            end
        end
        
        function fcnHandle = generateIKFunction(obj, functionName)
            %generateIKFunction Generate a MATLAB function to compute inverse kinematics in closed-form
            %   fcnHandle = generateIKFunction(a, functionName) generates a
            %   function that computes the closed-form solutions for
            %   inverse kinematics and returns it as a function handle.
            
            narginchk(2,2);
            
            % Check that the function is supported for IK
            if ~obj.IsValidGroupForIK
                robotics.manip.internal.error('analyticalinversekinematics:IKNotSupported');
            end
            
            % Input validation
            validateattributes(functionName, {'char','string'}, {'scalartext','nonempty'}, 'analyticalInverseKinematics', 'functionName');
            functionName = convertStringsToChars(functionName);
            
            % Generate inverse kinematics to file
            [~, fnName] = robotics.manip.internal.AnalyticalIKHelpers.generateIKcode(obj.SolutionDetails, functionName, obj.IdentificationTol, obj.SolutionTol);
            fcnHandle = str2func(fnName);
        end
    end
    
    % Set / Get methods
    methods
        function set.KinematicGroup(obj, kinGroup)
            %set.KinematicGroup Set method for KinematicGroup property
            
            % Validate the kinematic group
            obj.validateAndSetKinematicGroup(kinGroup);
            
            % Determine whether the group is supported for IK
            obj.SolutionDetails = obj.assessKinematicGroupForIK(obj.SubChain, obj.SubChainNumDoF);
        end
        
        function kinGroup = get.KinematicGroup(obj)
            %get.KinematicGroup Get method for KinematicGroup property
            
            kinGroup = obj.KinematicGroupInternal;
        end
        
        function kinGroupType = get.KinematicGroupType(obj)
            %get.KinematicGroup Get method for KinematicGroupType property
            %   Convert the classification details and subchain into a
            %   string-based user-facing format. The kinematic group
            %   classification type is a 1xN character array for the N
            %   non-fixed joints in the tree defined by the kinematic
            %   group, where the ith element of the string corresponds to
            %   the ith non-fixed joint. These values are either:
            %      'P'   - The joint is prismatic
            %      'S'   - The joint is revolute and its axis is one of
            %              three intersecting axes that comprise a
            %              spherical joint
            %      'R'   - The joint is revolute and its axis is not part
            %              of three consecutive, intersecting axes
            %
            %   If the value is empty, then the kinematic group defines a
            %   tree with no moving joints.
            
            kinGroupType = obj.formulateKinematicGroupTypeString(obj.SubChain, obj.SubChainNumDoF, obj.SolutionDetails);
        end
        
        function kinGroupConfigIdx = get.KinematicGroupConfigIdx(obj)
            %get.KinematicGroupConfigIdx Get method for KinematicGroupConfigIdx property
            
            % The subchain kinematic path is a vector where each element
            % indicates the index of the source body in the source
            % RigidBodyTree object. The first body is the base of the
            % subchain, which doesn't move. Since the subchain is a chain
            % (i.e. it doesn't have branches), the bodies are already in
            % the order of their configuration vector, though they do
            % include fixed bodies that will be filtered out downstream.
            movingBodyKinPath = obj.SubchainBodyIdMap(2:end);
            
            % The positionDoFMap relates the bodies in the rigidbodytree to
            % their indices in the associated configuration vector
            kinGroupBodyIdx = obj.RigidBodyTree.TreeInternal.PositionDoFMap(movingBodyKinPath,1)';
            
            % The positionDoF map uses "0" when the body is fixed (so
            % there's no corresponding configuration vector). Remove these
            % entries to only return indices that correspond to moving
            % bodies.
            kinGroupConfigIdx = kinGroupBodyIdx(kinGroupBodyIdx > 0);
        end
        
        function isValidIK = get.IsValidGroupForIK(obj)
            %get.IsValidGroupForIK Get method for IsValidGroupForIK property
            
            isValidIK = ~isempty(obj.SolutionDetails.SolutionMethod);
        end
        
        function newAIK = copy(obj)
            %copy Creates a deep copy of the analytical inverse kinematics object
            %   newAIK = copy(obj) creates a selective deep copy of the
            %   analyticalInverseKinematics object. Some internal properties
            %   will be reset to default instead of being copied (e.g.
            %   showdetails outputs).
            %
            %   Example:
            %       % Create an analytical inverse kinematics object
            %       robot = loadrobot('abbIrb120T');
            %       aik1 = analyticalInverseKinematics(robot);
            %
            %       % Make a deep copy
            %       aik2 = copy(aik1);
            %
            %   See also analyticalInverseKinematics
            
            % Create a new analytical IK object
            newAIK = analyticalInverseKinematics(...
                obj.RigidBodyTree, 'KinematicGroup', ...
                obj.KinematicGroupInternal);
            
            % Transfer the showdetails calculation details. The text will
            % need to be regenerated, since the object name will have
            % changed.
            newAIK.DetailsData.CalcDetails = obj.DetailsData.CalcDetails;
            
        end
    end
    
    methods (Static)
        function loadedAIK = loadobj(matFileObj)
            %loadobj Load object from MAT file
            
            % Create a new analytical IK object
            loadedAIK = analyticalInverseKinematics(...
                matFileObj.RigidBodyTree, 'KinematicGroup', ...
                matFileObj.KinematicGroupInternal);
            
            % Transfer the showdetails calculation details. The text will
            % need to be regenerated, since the object name will have
            % changed.
            loadedAIK.DetailsData.CalcDetails = matFileObj.DetailsData.CalcDetails;
            
        end
    end
    
    methods (Access = private)
        
        function initializeTreeProperties(obj, tree)
            %initializeTreeProperties Validate and initialize RigidBodyTree and affiliated properties
            
            % rigidBodyTree input validation
            validateattributes(tree, {'rigidBodyTree'}, {'nonempty'}, 'analyticalInverseKinematics', 'rigidBodyTree');
            
            % Initialize tree and key related properties
            obj.RigidBodyTree = copy(tree);
            obj.RigidBodyTree.DataFormat = 'column';
            
            obj.TreeNumDoF = tree.TreeInternal.VelocityNumber;
        end
        
        function kinGroup = getDefaultKinematicGroup(obj)
            %getDefaultKinematicGroup Populate default kinematic group for a given rigidBodyTree
            %   The default kinematic group is from the base to the last
            %   body in the tree.
            
            kinGroup = struct();
            kinGroup.BaseName = obj.RigidBodyTree.Base.Name;
            kinGroup.EndEffectorBodyName = obj.RigidBodyTree.TreeInternal.BodyNames{obj.RigidBodyTree.NumBodies};
        end
        
        function validateAndSetKinematicGroup(obj, kinGroup)
            
            % Verify that the input is a struct with the only the expected fields
            validateattributes(kinGroup, {'struct'}, {'nonempty', 'scalar'}, 'analyticalInverseKinematics', 'KinematicGroup');
            fields = fieldnames(kinGroup);
            cellfun(@(s)validatestring(s, {'BaseName', 'EndEffectorBodyName'}), fields, 'UniformOutput', false);
            
            % Individual validation for each field
            validateattributes(kinGroup.BaseName, {'string', 'char'}, {'nonempty'}, 'analyticalInverseKinematics', 'KinematicGroup.BaseName');
            validateattributes(kinGroup.EndEffectorBodyName, {'string', 'char'}, {'nonempty'}, 'analyticalInverseKinematics', 'KinematicGroup.EndEffectorBodyName');
            
            % Start by processing body names
            treeInternal = obj.RigidBodyTree.TreeInternal;
            baseName = convertStringsToChars(kinGroup.BaseName);
            eeName = convertStringsToChars(kinGroup.EndEffectorBodyName);
            
            % Validate body names against the actual tree
            baseID = treeInternal.validateInputBodyName(baseName);
            eeID = treeInternal.validateInputBodyName(eeName);
            if eeID == 0
                robotics.manip.internal.error('analyticalinversekinematics:EERobotBase');
            end
            
            % Get the kinematic path between the bodies and check that it's
            % valid. To ensure that this is a valid path, the base has to
            % be a parent of the end effector, which implies that it's in
            % the ancestor indices.
            eeParentIDs = treeInternal.ancestorIndices(treeInternal.Bodies{eeID});
            if ~any(baseID == eeParentIDs)
                robotics.manip.internal.error('analyticalinversekinematics:BaseNotInEEParentIndices');
            end
            newTreeKinPath = kinematicPath(treeInternal, baseName, eeName);
            
            % Assemble the sub-chain from the tree, which is a serial chain connecting the
            % base to the end effector. The subchain uses the kinematic
            % path, so there are no branches or leaf bodies.
            subChain = robotics.manip.internal.AnalyticalIKHelpers.buildNewChainFromKinematicGroup(treeInternal, newTreeKinPath);
            
            % Set Kinematic Group
            obj.KinematicGroupInternal = kinGroup;
            obj.SubChain = subChain;
            obj.SubchainBodyIdMap = newTreeKinPath;
            obj.SubChainNumDoF = obj.SubChain.TreeInternal.VelocityNumber;
        end
    end
    
    methods (Access = protected)
        
        function solutionDetails = assessKinematicGroupForIK(obj, subChain, subChainDoF)
            %assessKinematicGroupForIK Determine whether IK can be generated for the subChain defined by the kinematic group
            %   This method analyzes the rigid body tree that defines the
            %   subChain associated with the kinematic group. The method
            %   first determines the type (detailed by the joints and their
            %   intersections). If the type matches a supported 6-DoF robot
            %   (one where the joints are revolute and the last three
            %   joints intersect), then the method determines whether the
            %   non-intersecting joints can be parameterized using a format
            %   for which a solution can be generated. If the robot is
            %   supported, the SolutionMethod of the solutionDetails output
            %   will have a non-empty value.
            
            % Classify the robot type
            classificationDetails = robotics.manip.internal.AnalyticalIKHelpers.identifyRobotClassification(subChain, obj.IdentificationTol);
            
            % Initialize the solution details struct
            solutionDetails = struct();
            solutionDetails.ClassificationDetails = classificationDetails;
            solutionDetails.SolutionMethod = [];
            solutionDetails.SolutionMethodDetails = [];
            
            % Exit if robot is not 6-DoF
            if subChainDoF ~= 6
                return;
            end
            
            % Exit if the last three joints don't intersect
            if ~classificationDetails.IsIntersectingAxesMidpoint(5)
                return;
            end
            
            % Exit if robot contains prismatic joints
            if any(~classificationDetails.IsJointRevolute)
                return;
            end
            
            % Search for a set of DH parameters can be used to parameterize
            % the robot and provide a boolean that is true if so. The
            % dhSolDetails contains both the DH parameters and the
            % variables and parameters required to relate that
            % parameterization to the original one.
            [foundDHParams, dhSolDetails] = robotics.manip.internal.DHUtilities.searchForDHParameterization(subChain, obj.IdentificationTol);
                        
            % Exit if no valid DH parameters have been found
            if ~foundDHParams
                return;
            end
            
            % Compute the last three axes from the DH parameters, check
            % whether they are supported, and exit if not
            [lastThreeAxesSupported, dhSolDetails.LastThreeAxes] = robotics.manip.internal.AnalyticalIKHelpers.assessLastThreeJointsConfigurationForIKSupport(dhSolDetails.DHParams, obj.IdentificationTol);
            if ~lastThreeAxesSupported
                return;
            end
            
            % Eliminate unsupported or incorrect DH parameters
            isSupportedDHSubset = obj.filterUnsupportedDHParameters(dhSolDetails, subChain);
            
            if isSupportedDHSubset
                % If the DH parameters are valid and supported, update
                % the solution method to "DH", which indicates that
                % this robot can be parameterized using DH, and that the IK
                % problem can be solved for robots of this form.
                solutionDetails.SolutionMethod = 'DH';
                solutionDetails.SolutionMethodDetails = dhSolDetails;
            end
        end
        
        function validKinGroupDetails = getAllValidKinematicGroups(obj)
            %getAllValidKinematicGroups Compute all valid kinematic groups for a given rigidBodyTree
            
            % Initialize output
            validKinGroupDetails = struct('KinematicGroup', struct(), ...
                'Type', {}, ...
                'IsIntersectingAxesMidpoint', {}, ...
                'MidpointAxisIntersections', {});
            
            % Initialize index
            validKinGroupIdx = 1;
            
            % Starting at the base, search outward through the children and
            % find the indices of all the moving ancestor joints, and store
            % them in a cell array
            parentBody = obj.RigidBodyTree.Base;
            childBodies = parentBody.Children;
            
            % Initialize the outputs -- a logical array indicating whether
            % the associated joint is moving, and a cell array of the
            % ancestor body indices associated with moving joints. Both are
            % indexed by the bodyIndex. The first element
            % corresponds to the base.
            isMovingJointBody = false(obj.RigidBodyTree.NumBodies+1, 1);
            movingJtAncestorIndicesCell = cell(obj.RigidBodyTree.NumBodies+1,1);
            while ~isempty(childBodies)
                for i = 1:numel(childBodies)
                    % Get the body, its index, and the index of its parent
                    bodyToIndex = childBodies{i};
                    bodyIdx = bodyToIndex.BodyInternal.Index;
                    parentBodyIdx = bodyToIndex.BodyInternal.ParentIndex;
                    
                    % Determine whether the body is associated with a
                    % moving joint
                    isMovingJointBody(bodyIdx) = ~strcmpi(bodyToIndex.Joint.Type, 'fixed');
                    
                    % Catalog the moving joint ancestor indices for this
                    % body. Like the regular ancestor indices, these
                    % include the current joint (if it is a moving joint).
                    if parentBodyIdx == 0
                        % If the parent is the base, there are no moving
                        % joints in the ancestor bodies
                        movingJtAncestorIndices = [];
                    else
                        % Otherwise, inherit the moving joints from the
                        % parent
                        movingJtAncestorIndices = movingJtAncestorIndicesCell{parentBodyIdx,:};
                    end
                    
                    if isMovingJointBody(bodyIdx)
                        % If the current body is associated with a moving
                        % joint, append it to the moving joint ancestor
                        % indices
                        movingJtAncestorIndices = [bodyIdx movingJtAncestorIndices]; %#ok<AGROW>
                    end
                    
                    % Update the cell array of moving joint ancestor bodies
                    movingJtAncestorIndicesCell{bodyIdx} = movingJtAncestorIndices;
                    
                    % Check if the number of chain is at least 6 DoF
                    numDoFToBase = numel(movingJtAncestorIndices);
                    if numDoFToBase >= 6
                        % The base index will be 6 moving joints back from
                        % the current body.
                        firstMovingJointBodyIdx = movingJtAncestorIndices(6);
                        
                        % The base joint must actually be a joint that
                        % precedes the current first moving joint (to
                        % ensure that the moving joint does move, it can't
                        % also be the base). For example, if the robot has
                        % the form Base-L1-L2-...-L6 where each of the L
                        % bodies are attached to a joint, the robot from
                        % Base to L6 has 6-DoF, but the robot from L1 to L6
                        % has only 5-DoF, precisely because setting it as
                        % the base renders the joint fixed.
                        
                        % The goal here is to find all the bodies that
                        % could be the base. These include all the fixed
                        % joints that precede a moving joint, up to and
                        % including the next moving joint.
                        
                        % To explain this, if there are any fixed joints
                        % that precede a moving joint, they are candidates.
                        % For the previous example, suppose the robot has
                        % the form Base-B1-B2-L1-...-L6, where L1 to L6 are
                        % affixed to moving joints, and B1 and B2 are fixed
                        % joints attached to the base (L1 is attached to
                        % B2). Then valid 6-DoF robots include Base-L6, as
                        % well as B1-L6 and B2-L6, because these fixed
                        % joints precede the first moving joint. However,
                        % one cannot simply take the ancestor indices of
                        % the current joint, as these may include other
                        % moving joints. To return to the example, suppose
                        % the robot is of the form Base-L1-...-L7. Then
                        % acceptable 6-DoF robots are defined from Base-L6,
                        % or from L1-L7. Now if we add  an intermediate
                        % fixed bodies L1fixed, such that the robot has the
                        % form Base-L1-L1fixed-L2-...-L7, then valid
                        % configurations are Base-L6, L1-L7, and
                        % L1fixed-L7, precisely because L1fixed and L1 are
                        % the ancestor indices of L2 up to and including
                        % the next moving joint, L1. Therefore, we instead
                        % get the first moving joint (L2 in the example)'s
                        % ancestor bodies, and then filter out all those
                        % after the next moving joint (L1 in the example).
                        firstJointBodyAncestorIndices = obj.RigidBodyTree.TreeInternal.ancestorIndices(obj.RigidBodyTree.TreeInternal.Bodies{firstMovingJointBodyIdx});
                        
                        % Exclude the first moving joint from the list of
                        % candidate body indices for the base
                        possibleBaseIndices = firstJointBodyAncestorIndices(2:end);
                        
                        if numDoFToBase == 6
                            % If the current chain has six moving joints,
                            % then all bodies before the first moving joint
                            % are either fixed or the base, which means
                            % that all it preceding ancestor bodies are
                            % valid base bodies
                            validBaseIndices = possibleBaseIndices;
                        else
                            % If the base index is a later one in the list,
                            % it is necessary to find all the fixed bodies
                            % up to the previous moving joint. For clarity,
                            % the notation here uses "position" to indicate
                            % the position of a value in a vector, and
                            % "index" when the value corresponds to a body
                            % index (rather than referring to them both as
                            % indices).
                            
                            % Compute the body index of the seventh moving
                            % joint back from the current body, which lies
                            % somewhere between the valid base in the sixth
                            % position, and the base
                            prevMovingJointIndex = movingJtAncestorIndices(7);
                            
                            % Compute the position of the body associated with
                            % the previous moving joint in the vector of
                            % possible base body indices
                            prevJointBodyIdxPosition = find(possibleBaseIndices == prevMovingJointIndex, 1);
                            
                            % The valid body indices for the base will be
                            % all the bodies in the ancestors between the
                            % first body and the previous moving joint body
                            validBaseIndices = possibleBaseIndices(1:prevJointBodyIdxPosition);
                        end
                        
                        % Now have a matrix where the rows indicate base
                        % and end effector body indices that form a 6-DoF
                        % robot. To be valid for IK, we have to further
                        % check how each pair is parameterized
                        for kinGroupBodyPairIdx = 1:numel(validBaseIndices)
                            % Check if the body pair is supported
                            kinGroupDetails = obj.assessKinGroupBodyPairForIK(validBaseIndices(kinGroupBodyPairIdx), bodyIdx);
                            
                            if ~isempty(kinGroupDetails)
                                validKinGroupDetails(validKinGroupIdx) = kinGroupDetails;
                                validKinGroupIdx = validKinGroupIdx + 1;
                            end
                        end
                    end
                    
                    
                    % Get all the children of this body and add them to the
                    % cell array of bodies to be searched
                    childBodies = [childBodies bodyToIndex.Children]; %#ok<AGROW>
                end
                
                % Remove the bodies that have just been assessed
                childBodies(1:i) = [];
            end
        end
        
        function kinGroupDetails = assessKinGroupBodyPairForIK(obj, baseIdx, eeBodyIdx)
            %assessKinGroupBodyPairForIK Check whether two bodies form a kinematic chain that is valid for IK
            %   This function accepts two body indices, corresponding to
            %   the base and end effector of the rigid body tree, and
            %   checks whether these form a subchain for which IK can be
            %   generated. The function returns the details of the
            %   numerical assessment.
            
            % Get the base and end effector names
            if baseIdx == 0
                baseName = obj.RigidBodyTree.BaseName;
            else
                baseName = obj.RigidBodyTree.BodyNames{baseIdx};
            end
            eeName = obj.RigidBodyTree.BodyNames{eeBodyIdx};
            
            % Define a subchain from these bodies
            newTreeKinPath = kinematicPath(obj.RigidBodyTree.TreeInternal, baseName, eeName);
            subChain = robotics.manip.internal.AnalyticalIKHelpers.buildNewChainFromKinematicGroup(obj.RigidBodyTree.TreeInternal, newTreeKinPath);
            
            % Assess the subchain for inverse kinematics
            ikAssessmentDetails = obj.assessKinematicGroupForIK(subChain, 6);
            
            % If a valid solution is produced, add it to the output
            % display
            if ~isempty(ikAssessmentDetails.SolutionMethod)
                % Update the output struct
                calculationDetails = struct();
                calculationDetails.KinematicGroup = struct('BaseName', baseName, 'EndEffectorBodyName', eeName);
                calculationDetails.Type = obj.formulateKinematicGroupTypeString(subChain, numel(ikAssessmentDetails.ClassificationDetails.IsJointRevolute), ikAssessmentDetails);
                calculationDetails.IsIntersectingAxesMidpoint = ikAssessmentDetails.ClassificationDetails.IsIntersectingAxesMidpoint;
                calculationDetails.MidpointAxisIntersections = ikAssessmentDetails.ClassificationDetails.MidpointAxisIntersections;
                
                kinGroupDetails = calculationDetails;
            else
                kinGroupDetails = [];
            end
            
        end
        
        function displayWidths = initializeShowDetailsOutput(obj)
            %initializeShowDetailsOutput Initialize the showdetails output
            %   This function initializes obj.DetailsData.DisplayText by
            %   defining the header lines for the showdetails output.
            %   Before this method is called, obj.DetailsData.DisplayText is
            %   empty. Additionally, the method outputs display widths,
            %   which are a vector of widths that ensure that each element
            %   in the table will be properly spaced. These widths are used
            %   downstream when lines are added to the display output.
            
            breakLine = sprintf('--------------------\n');
            headerText = sprintf('Robot: (%d bodies)\n\n', int32(obj.RigidBodyTree.NumBodies));
            
            % Set the maximum widths for each of the categories to display
            
            % Since only 6-DoF robots are supported for solutions, the type
            % is the maximum of 6 (the length of the type string) or the
            % length of the header text
            typeTitleWidth = numel(message('robotics:robotmanip:analyticalinversekinematics:ShowdetailsTypeHeaderText').getString);
            maxTypeWidth = max(6, typeTitleWidth);
            
            % The width of the index column corresponds to the maximum of
            % the number of digits in the max number of bodies (computed
            % using the log10) or the length of the header text
            idxTitleWidth = numel(message('robotics:robotmanip:analyticalinversekinematics:ShowdetailsIdxHeaderText').getString);
            maxNumBodiesWidth = floor(log10(obj.RigidBodyTree.NumBodies)) + 1;
            idxWidth = max(maxNumBodiesWidth, idxTitleWidth);
            
            % The width of the actions column is the maximum of the "use
            % kinematic group" link text or the header text
            actionsTitleWidth = numel(message('robotics:robotmanip:analyticalinversekinematics:ShowdetailsActionsHeaderText').getString);
            actionsWidth = max(numel(message('robotics:robotmanip:analyticalinversekinematics:UseKinematicGroupLinkText').getString), ...
                actionsTitleWidth);
            
            % Initialize the body name size to either the length of the
            % title line, or the base name -- whichever is larger. Then
            % iterate over the body names in case there are any that are
            % larger still. Note that, since these bodies aren't
            % necessarily used in the output, this can lead to extra large
            % margins in some cases.
            baseNameTitleWidth = numel(message('robotics:robotmanip:analyticalinversekinematics:ShowdetailsBaseNameHeaderText').getString);
            eeBodyNameTitleWidth = numel(message('robotics:robotmanip:analyticalinversekinematics:ShowdetailsEEBodyNameHeaderText').getString);
            maxBodyNameWidth = max(eeBodyNameTitleWidth, length(obj.RigidBodyTree.BaseName));
            for i = 1:obj.RigidBodyTree.NumBodies
                maxBodyNameWidth = ...
                    max(maxBodyNameWidth, length(obj.RigidBodyTree.Bodies{i}.Name)+5);
            end
            
            % Store the display widths so they can be passed to the line
            % entry functions
            displayWidths = [idxTitleWidth maxBodyNameWidth maxTypeWidth actionsWidth];
            
            titlesText = sprintf('%*s   %*s   %*s   %*s   %*s\n', ...
                idxWidth, message('robotics:robotmanip:analyticalinversekinematics:ShowdetailsIdxHeaderText').getString, ...
                maxBodyNameWidth, message('robotics:robotmanip:analyticalinversekinematics:ShowdetailsBaseNameHeaderText').getString,...
                maxBodyNameWidth, message('robotics:robotmanip:analyticalinversekinematics:ShowdetailsEEBodyNameHeaderText').getString,...
                maxTypeWidth, message('robotics:robotmanip:analyticalinversekinematics:ShowdetailsTypeHeaderText').getString,...
                actionsWidth, message('robotics:robotmanip:analyticalinversekinematics:ShowdetailsActionsHeaderText').getString);
            
            titlesLines = sprintf('%*s   %*s   %*s   %*s   %*s\n', ...
                idxWidth, repmat('-',1,idxTitleWidth), ...
                maxBodyNameWidth, repmat('-',1,baseNameTitleWidth),...
                maxBodyNameWidth, repmat('-',1,eeBodyNameTitleWidth),...
                maxTypeWidth, repmat('-',1,typeTitleWidth),...
                actionsWidth, repmat('-',1,actionsTitleWidth));
            
            obj.DetailsData.DisplayText = [breakLine ...
                headerText ...
                titlesText ...
                titlesLines];
        end
        
        function addDisplayDetailsLine(obj, displayWidths, objName, entryIdx, calcDetails)
            %addDisplayDetailsLine Append an entry to the showdetails display
            %   This method is called to update obj.DetailsData.DisplayText,
            %   which contains the string that is output when showdetails
            %   is called. This method is primarily used to convert the
            %   details stored in calcDetails to the line at that entryIdx.
            %   Additionally, the function accepts the object name for use
            %   in link command creation, as well as a vector of display
            %   widths that are used to ensure that the line text is
            %   correctly spaced.
            
            % Extract display widths
            idxWidth = displayWidths(1);
            maxBodyNameWidth = displayWidths(2);
            maxTypeWidth = displayWidths(3);
            actionsWidth = displayWidths(4);
            
            indexText = sprintf('%*d', idxWidth, int32(entryIdx));
            baseNameText = sprintf('   %*s', maxBodyNameWidth, calcDetails.KinematicGroup.BaseName);
            eeNameText = sprintf('   %*s', maxBodyNameWidth, calcDetails.KinematicGroup.EndEffectorBodyName);
            typeText = sprintf('   %*s', maxTypeWidth, calcDetails.Type);
            
            %Links have more fprintf length than they take up on screen
            linkCommandsString = [...
                sprintf('if exist(''%s'', ''var'') && isa(%s, ''analyticalInverseKinematics''), ', objName, objName), ...
                sprintf('%s.KinematicGroup = struct(''BaseName'', ''%s'', ''EndEffectorBodyName'', ''%s''); ', objName, calcDetails.KinematicGroup.BaseName, calcDetails.KinematicGroup.EndEffectorBodyName), ...
                sprintf('fprintf(''Applying showdetails option %i to %s.KinematicGroup:\\n''); ', entryIdx, objName), ...
                sprintf('disp(%s.KinematicGroup); ', objName), ...
                sprintf('else, '), ...
                sprintf('robotics.manip.internal.error(''analyticalinversekinematics:ObjectDeleted'', ''%s'');', objName), ...
                sprintf('end')
                ];
            actionLink = sprintf('<a href="matlab:%s">%s</a>', linkCommandsString, message('robotics:robotmanip:analyticalinversekinematics:UseKinematicGroupLinkText').getString);
            actionsText = sprintf('   %*s', actionsWidth, actionLink);
            
            entriesText = [indexText baseNameText eeNameText ...
                typeText actionsText newline];
            
            % Add to existing display text
            obj.DetailsData.DisplayText = [obj.DetailsData.DisplayText entriesText];
        end
        
        function isValidDHExtraction = checkRobotDHExtraction(obj, sourceRobot, conversionDetails)
            %checkRobotDHExtraction Check that DH Parameter extraction is valid
            %   This function verifies that DH parameters have been
            %   correctly extracted from a source robot. The method accepts
            %   the rigidBodyTree of the source robot, as well as
            %   conversionDetails, a structure which includes the DH
            %   Parameters and Z-Screws that have been extracted from the
            %   tree using the conversion tools. The method then computes
            %   the pose of the last body for both the source robot and
            %   a robot constructed using the derived DH parameters /
            %   z-screws for two configurations, the home configuration and
            %   a random configuration, and checks whether these poses are
            %   equal within a predefined tolerance. If the DH parameters
            %   have been correctly extracted, both checks should pass; if
            %   not, one or both could fail. If both checks pass, the
            %   method returns TRUE; otherwise isValidDHExtraction returns
            %   FALSE. The primary goal of this method is to ensure that
            %   the extracted DH parameters are valid, thereby ensuring a
            %   valid generated solution.
            
            % Build a robot using DH parameters
            robotDH = robotics.manip.internal.robotplant.revoluteDHRobot(conversionDetails.DHParams);
            eeTform = conversionDetails.F6ToEETform;
            baseTform = conversionDetails.WorldToDHBase;
            
            % Compute the pose with the home configuration
            expConfig1 = sourceRobot.homeConfiguration;
            expConfigDH1 = expConfig1 + conversionDetails.ZScrews(1:end,2);
            expEEPose1 = sourceRobot.getTransform(expConfig1, sourceRobot.Bodies{end}.Name);
            expEEPoseDH1 = baseTform*robotDH.getTransform(expConfigDH1, robotDH.Bodies{end}.Name)*eeTform;
            homeDiffMat = abs(expEEPose1 - expEEPoseDH1);
            passHomeConfigCheck = all(all(homeDiffMat < obj.SolutionTol));
            
            % Compute the pose with a random configuration.
            expConfig2 = sourceRobot.randomConfiguration;
            expConfigDH2 = expConfig2 + conversionDetails.ZScrews(1:end,2);
            expEEPose2 = sourceRobot.getTransform(expConfig2, sourceRobot.Bodies{end}.Name);
            expEEPoseDH2 = baseTform*robotDH.getTransform(expConfigDH2, robotDH.Bodies{end}.Name)*eeTform;
            randomDiffMat = abs(expEEPose2 - expEEPoseDH2);
            passRandomConfigCheck = all(all(randomDiffMat < obj.SolutionTol));
            
            isValidDHExtraction = passHomeConfigCheck && passRandomConfigCheck;
        end
        
        function isSupportedDHParameterSet = filterUnsupportedDHParameters(obj, conversionDetails, subChain)
            %filterUnsupportedDHParameters Check DH Parameters and remove sets that are explicitly unsupported
            %   This function assesses the DH Parameters that are extracted
            %   from a robot and returns a flag that is true if those
            %   parameters are supported and false otherwise. Note that
            %   this method does not check for intersecting axes or 6-DoF,
            %   as those items have already been verified before DH
            %   parameters were extracted. Instead, this method is used as
            %   a filter to remove extracted DH Parameter sets for which
            %   the feature's generation tools cannot provide universally
            %   applicable solutions. Additionally, the method checks that
            %   the derived robot, subChain, actually matches the robot
            %   defined by the extracted DH parameters.
            
            % Extract DH Parameters from conversion details
            dhParams = conversionDetails.DHParams;
            
            % Default to TRUE (parameters are supported)
            isSupportedDHParameterSet = true; %#ok<NASGU>
            
            % Case: alpha1 is zero, so equations 3.25 and 3.26 from
            % [Pieper, 1968] are solved separately, but theta2 and theta3
            % are parallel yet not co-axial. In that case, equation 3.26
            % becomes trivial (any value of theta3 will solve it), but
            % equation 3.25 is not trivial and constrains the solution.
            % This happens when a2 and d2 are nonzero but alpha2 is zero.
            a2 = dhParams(2,1);
            alpha1 = dhParams(1,2);
            alpha2 = dhParams(2,2);
            if abs(a2) > obj.IdentificationTol ...
                    && abs(alpha1) < obj.IdentificationTol && abs(alpha2) < obj.IdentificationTol
                	isSupportedDHParameterSet = false;
                return;
            end
            
            % Run a forward kinematics check that verifies that the
            % extracted DH parameters correctly represent the source
            % robot. This check adds some performance overhead, so it's
            % important to only run it in the cases where previous
            % conversion checks have already passed and DH parameter
            % extraction has already succeeded.
            isSupportedDHParameterSet = obj.checkRobotDHExtraction(subChain, conversionDetails);
        end
    end
    
    methods (Static, Access = protected)
        
        
        function kinGroupType = formulateKinematicGroupTypeString(subChain, subChainNumDoF, solutionDetails)
            %formulateKinematicGroupTypeString Convert classification details into character array output
            %   Convert the classification details and subchain into a
            %   string-based user-facing format. The kinematic group
            %   classification type is a 1xN character array for the N
            %   non-fixed joints in the tree defined by the kinematic
            %   group, where the ith element of the string corresponds to
            %   the ith non-fixed joint. These values are either:
            %      'P'   - The joint is prismatic
            %      'S'   - The joint is revolute and its axis is one of
            %              three intersecting axes that comprise a
            %              spherical joint
            %      'R'   - The joint is revolute and its axis is not part
            %              of three consecutive, intersecting axes
            %
            %   If the value is empty, then the kinematic group defines a
            %   tree with no moving joints.
            
            kinGroupType = '';
            for jointIdx = 1:subChainNumDoF
                jointBodyIndex = robotics.manip.internal.AnalyticalIKHelpers.getBodyIndexFromJointIndex(subChain, jointIdx);
                
                switch subChain.Bodies{jointBodyIndex}.Joint.Type
                    case 'revolute'
                        % Check if this joint or either of the adjacent
                        % joints is the center of three intersecting joints
                        if jointIdx > 1
                            prevJointIdx = jointIdx-1;
                        else
                            prevJointIdx = [];
                        end
                        if jointIdx < subChainNumDoF
                            nextJointIdx = jointIdx+1;
                        else
                            nextJointIdx = [];
                        end
                        isIntersectingAxisJoint = any(solutionDetails.ClassificationDetails.IsIntersectingAxesMidpoint([prevJointIdx jointIdx nextJointIdx]));
                        
                        if isIntersectingAxisJoint
                            kinGroupType(jointIdx) = 'S';
                        else
                            kinGroupType(jointIdx) = 'R';
                        end
                    case 'prismatic'
                        kinGroupType(jointIdx) = 'P';
                end
            end
        end
    end
end
