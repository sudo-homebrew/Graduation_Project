classdef interactiveRigidBodyTree < robotics.manip.internal.InternalAccess ...
        & robotics.core.internal.mixin.Unsaveable
    %INTERACTIVERIGIDBODYTREE Interact with rigid body tree robot models
    %   The interactiveRigidBodyTree object creates a figure that displays
    %   a rigidBodyTree robot model and enables you to directly modify the
    %   robot position and configuration using interactive markers. You can
    %   change the desired end effector by right-clicking different bodies.
    %   Click and drag the interactive markers to compute new
    %   configurations using inverse kinematics.
    %
    %   VIZTREE = interactiveRigidBodyTree(tree) creates an interactive
    %   tree object and associated figure for the specified rigidBodyTree
    %   object.
    %
    %   VIZTREE = interactiveRigidBodyTree(___, 'PropertyName', PropertyValue, ..)
    %   sets additional properties specified as name-value pairs. You can
    %   specify multiple properties in any order.
    %
    %   INTERACTIVERIGIDBODYTREE Properties:
    %      SolverPoseWeights      - Weights on orientation and position elements of target pose
    %      Constraints            - Cell array of constraints on inverse kinematics
    %      MarkerBodyName         - Associated rigid body name for interactive marker
    %      MarkerControlMethod    - Type of interactive control for body
    %      MarkerScaleFactor      - A scalar indicating the relative scale of the marker
    %      ShowMarker             - Boolean flag indicating whether to show marker
    %      Configuration          - Current configuration of rigidBodyTree robot model
    %      StoredConfigurations   - Array of stored robot configurations
    %
    %   Properties that can only be user-defined at construction:
    %      RigidBodyTree          - Rigid body tree robot model
    %      IKSolver               - Inverse kinematics solver object
    %
    %   Read-only properties:
    %      MarkerPose             - Pose of interactive marker
    %      MarkerBodyPose         - Pose of rigid body associated with interactive marker
    %
    %   VIZTREE = interactiveRigidBodyTree(___, 'Name', Value, ..) defines
    %   additional attributes specified as name-value pairs. Name-value
    %   pairs include:
    %      
    %      Frames                - A string or character vector to display 
    %                              body frames, specified as "on" or "off".
    %                              (Default: "on")
    %
    %   INTERACTIVERIGIDBODYTREE object functions:
    %      addConfiguration      - Add current configuration to StoredConfigurations property
    %      removeConfigurations  - Clear configurations from StoredConfigurations property
    %      addConstraint         - Add inverse kinematics constraint
    %      removeConstraints     - Remove inverse kinematics constraints
    %      showFigure            - Show the figure and set it to the current axis
    %
    %   Example:
    %      % Load robot
    %      robot = loadrobot("atlas");
    %
    %      % Create an interactive rigidBodyTree object to move the arm around
    %      viztree = interactiveRigidBodyTree(robot, "MarkerBodyName", "l_lfarm");
    %
    %      % Add a constraint to respect joint limits
    %      jtCon = constraintJointBounds(robot);
    %      addConstraint(viztree, jtCon);
    %
    %   See also inverseKinematics, generalizedInverseKinematics,
    %   rigidBodyTree
    
    %   Copyright 2019-2021 The MathWorks, Inc.
    
    properties (Dependent, SetAccess = private)
        
        %RigidBodyTree - Rigid body tree robot model
        %   Rigid body tree robot model, specified as a rigidBodyTree
        %   object that defines the inertial and kinematic properties of
        %   the manipulator. The RigidBodyTree property is defined at
        %   object construction and is read-only.
        RigidBodyTree
        
        %IKSolver - Inverse kinematics solver object
        %   Inverse kinematics solver object, specified as a
        %   generalizedInverseKinematics object with the associated
        %   rigidBodyTree object from the RigidBodyTree property. When this
        %   property is defined at input, the solver object in the
        %   interactiveRigidBodyTree object is created via a deep copy. By
        %   default, the solver method is "Levenberg-Marquardt" with
        %   MaxIterations property set to 2. Increasing MaxIterations can
        %   significantly impact frame rate in the figure.
        IKSolver
    end
    
    properties (Dependent)
        
        %SolverPoseWeights - Weights on orientation and position elements of target pose
        %   The solver pose weights property is a 2-element vector that
        %   applies weights to the difference in orientation and position
        %   of the inverse kinematics solution relative to the interactive
        %   marker. The values correspond to the Weights property for a
        %   constraintPoseTarget object.
        %   Default: [1 1]
        SolverPoseWeights
        
        %Constraints - Cell array of constraints on inverse kinematics
        %   Constraint objects provided to the inverse kinematics solver as a
        %   cell array of objects. Possible elements include objects of type
        %   constraintOrientationTarget, constraintPoseTarget,
        %   constraintPositionTarget, constraintCartesianBounds,
        %   constraintAiming, constraintRevoluteJoint,
        %   constraintPrismaticJoint, constraintFixedJoint, and
        %   constraintJointBounds. Note that with the default value for
        %   IKSolver, joint limits will be respected via the solver, regardless
        %   of the absence of an explicit constraint.
        %   Default: {}
        Constraints
        
        %MarkerBodyName - Associated rigid body name for interactive marker
        %   A string that corresponds to the name of a body on the
        %   associated rigidBodyTree object. The marker is placed at the
        %   origin of the rigidBody with a corresponding name. When the
        %   marker is enabled but no marker body name has been set, the
        %   last value of the rigidBodyTree bodynames is used as the
        %   default value.
        MarkerBodyName
        
        %MarkerControlMethod - Type of interactive control for body
        %   A string that defines the type of interactive control assigned
        %   to be marker body. There are two valid options:
        %      - "InverseKinematics": The marker body is treated as the end-effector of
        %      the rigid body tree and inverse kinematics is used to
        %      resolve the position of the associated joints.
        %
        %      - "JointControl": The marker body is used to adjust the
        %      associated link only. Use the marker to directly control the
        %      position of the associated joint.
        MarkerControlMethod
        
        %MarkerScaleFactor - A scalar indicating the relative scale of the marker
        %   Default: 1
        MarkerScaleFactor
        
        %ShowMarker - Boolean flag indicating whether to show marker
        %   The default value is FALSE. When the MarkerControlMethod or
        %   MarkerBodyName properties are provided at construction, or when
        %   they are updated, the value of ShowMarker is set to TRUE.
        ShowMarker
        
        %Configuration - Current configuration of rigidBodyTree robot model
        %   An N-element vector of joint values, where N is the number of
        %   joints in the associated rigidBodyTree object, that defines
        %   the robot configuration at a given instant. 
        %   Default: Home configuration of the associated rigidBodyTree.
        Configuration
        
        %StoredConfigurations - Array of stored robot configurations
        %   An NxP array of P N-element joint configurations. The
        %   StoredConfigurations matrix can be queried directly, or can be
        %   updated using the associated utility functions:
        %   addConfiguration and removeConfigurations. 
        %   Default: []
        StoredConfigurations
    end
    
    properties (Dependent, SetAccess = private)
        %MarkerPose - Pose of interactive marker
        %   A 4x4 homogeneous transform that indicates the pose of the
        %   marker at a given instant. This property is read-only.
        MarkerPose
        
        %MarkerBodyPose - Pose of rigid body associated with interactive marker
        %   A 4x4 homogeneous transform that indicates the pose of the
        %   marker body at a given instant. This property is read-only.
        MarkerBodyPose
    end
    
    properties (Constant, Access=?robotics.manip.internal.InternalAccess)
        
        % Set defaults that are not dependent on any other properties
        
        SolverPoseWeightsDefault = [1 1]
        
        ConstraintsDefault = {}
        
        MarkerControlMethodDefault = 'InverseKinematics'
        
        MarkerScaleFactorDefault = 1
        
        ShowMarkerDefault = true
        
        StoredConfigurationsDefault = []
        
        FramesDefault = 'on'
    end
    
    properties (Access=?robotics.manip.internal.InternalAccess)
        
        % Visualization helper
        VisualizationHelper
        
        % InteractiveRBTBackend object
        Backend
    end
    
    methods
        function obj = interactiveRigidBodyTree(tree, varargin)
            %interactiveRigidBodyTree Object constructor
            
            % rigidBodyTree input validation
            narginchk(1, 19);
            validateattributes(tree, {'rigidBodyTree'}, {'nonempty'}, 'interactiveRigidBodyTree', 'rigidBodyTree');
            
            % Initialize tree and key related properties
            rigidBodyTree = copy(tree);
            rigidBodyTree.DataFormat = 'column';
            
            % Process options specified in constructor
            charInputs = cell(1,nargin-1);
            [charInputs{:}] = convertStringsToChars(varargin{:});
            names = {...
                'IKSolver', ...
                'SolverPoseWeights', ...
                'Constraints', ...
                'MarkerBodyName', ...
                'MarkerScaleFactor', ...
                'ShowMarker', ...
                'MarkerControlMethod', ...
                'Configuration', ...
                'StoredConfigurations', ...
                'Frames', ...
                };
            defaults = {...
                [], ...
                obj.SolverPoseWeightsDefault, ...
                obj.ConstraintsDefault, ...
                [], ...
                obj.MarkerScaleFactorDefault, ...
                [], ...
                [], ...
                rigidBodyTree.homeConfiguration, ...
                [], ...
                obj.FramesDefault, ...
                };
            
            % Parse inputs
            parser = robotics.core.internal.NameValueParser(names, defaults);
            parse(parser, charInputs{:});
            
            % Initialize IK Solver and associated properties
            [ikSolver, solverPoseWeights, ikConstraints] = obj.initializeIKProperties(rigidBodyTree, parser, names{1}, names{2}, names{3});
            
            % Initialize marker properties
            [markerBodyName, markerScaleFactor, showMarker, markerControlMethod] = obj.initializeMarkerProperties(rigidBodyTree, parser, names{4}, names{5}, names{6}, names{7});
            
            % Initialize other properties
            config = obj.validateConfiguration(rigidBodyTree, parameterValue(parser, names{8}));
            storedConfigs = obj.validateStoredConfigurations(rigidBodyTree, parameterValue(parser, names{9}));
            frameStatus = validatestring(parameterValue(parser, names{10}), {'on', 'off'}, 'interactiveRigidBodyTree', names{10});
            
            % Initialize backend object
            obj.Backend = robotics.manip.internal.InteractiveRBTBackend(rigidBodyTree, ikSolver, ...
                solverPoseWeights, ikConstraints, ...
                markerBodyName, markerControlMethod, markerScaleFactor, showMarker, ...
                config, storedConfigs, frameStatus);
            
            % Initialize figure visualization
            obj.VisualizationHelper = robotics.manip.internal.InteractiveVisualizationHelper(obj.Backend);
        end
        
        function delete(obj)
            %delete Destructor
            
            delete(obj.VisualizationHelper);
            delete(obj.Backend);
            
        end
        
        function addConfiguration(obj, idx)
            %addConfiguration Add current configuration to StoredConfigurations property            %
            %   addConfiguration(OBJ) appends the current configuration to 
            %   the matrix in the StoredConfigurations property.
            %
            %   addConfiguration(OBJ, IDX) inserts the current
            %   configuration into the StoredConfigurations property at the index specified by IDX.
            %
            %   Example:
            %      % Load a Valkyrie robot and create an
            %      % interactiveRigidBodyTree visualization
            %      robot = loadrobot("valkyrie");
            %      interactiveViz = interactiveRigidBodyTree(robot);
            %
            %      % Set the configuration to a random configuration and
            %      % add it to the StoredConfigurations property matrix
            %      interactiveViz.Configuration = randomConfiguration(interactiveViz.RigidBodyTree);
            %      addConfiguration(interactiveViz);
            %
            %      % Set the configuration to the robot's home configuration
            %      interactiveViz.Configuration = homeConfiguration(interactiveViz.RigidBodyTree);
            %
            %      % Insert the homeConfiguration into the
            %      % StoredConfigurations matrix in the first index
            %      addConfiguration(interactiveViz,1);
            
            if nargin < 2
                % Append to the end
                obj.StoredConfigurations(:,end+1) = obj.Configuration;
            else
                % Insert before the current IDX
                validateattributes(idx, {'numeric'}, {'nonempty', 'nonnan', 'finite', 'integer', 'scalar', 'positive'}, 'addConfiguration', 'idx');
                obj.StoredConfigurations = [obj.StoredConfigurations(:,1:(idx-1)) obj.Configuration obj.StoredConfigurations(:, (idx):end)];
            end
        end
        
        function removeConfigurations(obj, idx)
            %removeConfiguration Clear configurations from StoredConfigurations property
            %   removeConfigurations(OBJ) removes the last column from the
            %   matrix of configurations in the StoredConfigurations
            %   property.
            %
            %   removeConfigurations(OBJ, IDX) removes the columns of the
            %   StoredConfigurations property at the indices specified by
            %   IDX. The values of IDX must be less than or equal to the
            %   number of columns in the matrix.
            %
            %   Example:
            %      % Load a Baxter robot and create an
            %      % interactiveRigidBodyTree visualization
            %      robot = loadrobot("rethinkBaxter");
            %      interactiveViz = interactiveRigidBodyTree(robot, 'ShowMarker', true);
            %
            %      % Set the configuration to a random configuration and
            %      % add it to the StoredConfigurations property matrix
            %      interactiveViz.Configuration = randomConfiguration(interactiveViz.RigidBodyTree);
            %      addConfiguration(interactiveViz);
            %
            %      % Set the configuration to the robot's home configuration
            %      interactiveViz.Configuration = homeConfiguration(interactiveViz.RigidBodyTree);
            %
            %      % Insert the homeConfiguration into the
            %      % StoredConfigurations matrix in the first index
            %      addConfiguration(interactiveViz,1);
            %      
            %      % Append the homeConfiguration to the end of the
            %      % StoredConfigurations matrix
            %      addConfiguration(interactiveViz,1);
            %
            %      % Remove the first two configurations in the
            %      % StoredConfigurations property value
            %      removeConfigurations(interactiveViz,1:2);
            
            numStoredConfigs = size(obj.StoredConfigurations, 2);
            
            if nargin < 2
                % Set the index to the last column
                idx = numStoredConfigs;
            else
                % Validate input index
                validateattributes(idx, {'numeric'}, {'nonempty', 'nonnan', 'finite', 'integer', 'vector', 'positive','<=', numStoredConfigs, 'increasing'}, 'removeConfigurations', 'idx');
            end
            
            % Remove only the indices specified by IDX
            obj.StoredConfigurations = obj.StoredConfigurations(:, setdiff(1:numStoredConfigs, idx));
            
            % If individual columns are set to empty and this renders the
            % whole matrix empty, it will actually be an Nx0 empty matrix.
            % To avoid this issue, clean up and ensure that all empty
            % variants have the value [].
            if isempty(obj.StoredConfigurations)
                obj.StoredConfigurations = [];
            end
        end
        
        function addConstraint(obj, gikConstraint)
            %addConstraint Add inverse kinematics constraint
            %   addConstraint(OBJ, GIKCONSTRAINT) adds a constraint object
            %   to the Constraints property. The constraint must be one of
            %   the supported GIK constraint objects.
            %
            %   Example:
            %      % Load robot
            %      robot = loadrobot("atlas");
            %
            %      % Create an interactive rigidBodyTree object to move the arm around
            %      viztree = interactiveRigidBodyTree(robot, "MarkerBodyName", "l_lfarm");
            %
            %      % Add a constraint to respect joint limits
            %      jtCon = constraintJointBounds(robot);
            %      addConstraint(viztree, jtCon);
            
            obj.Constraints = [obj.Constraints {gikConstraint}];
        end
        
        function removeConstraints(obj, idx)
            %removeConstraint Remove constraints from the Constraints property
            %   removeConstraints(OBJ) removes the last constraint from the
            %   cell array of constraints in the Constraints property of
            %   the interactiveRigidBodyTree object.
            %
            %   removeConstraints(OBJ, IDX) removes the constraints from
            %   the cell array of constraints in the Constraints property
            %   of the interactiveRigidBodyTree object at the indices
            %   specified by IDX.
            %
            %   Example:
            %      % Load robot
            %      robot = loadrobot("atlas");
            %
            %      % Create an interactiveRigidBodyTree object with joint
            %      % limit constraints
            %      jtCon = constraintJointBounds(robot);
            %      viztree = interactiveRigidBodyTree(robot, "Constraints", {jtCon});
            %
            %      % Remove the joint constraints
            %      removeConstraints(viztree);
            
            if nargin < 2
                obj.Constraints(end) = [];
            else
                validateattributes(idx, {'numeric'}, {'nonempty', 'nonnan', 'finite', 'integer', 'vector', 'positive'}, 'removeConstraints', 'idx');
                obj.Constraints(idx) = [];
            end
        end
        
        function showFigure(obj)
            %showFigure Show the figure and set it to the current axis
            %   showFigure(OBJ) makes the associated figure visible and
            %   brings it into focus.
            %
            %   Example:
            %      % Load robot
            %      robot = loadrobot("atlas");
            %
            %      % Create an interactiveRigidBodyTree object
            %      viztree = interactiveRigidBodyTree(robot);
            %      
            %      % Add a collision object to the figure
            %      ax = gca;
            %      cBox = collisionBox(1,1,0.1);
            %      show(cBox, 'Parent', ax);
            %
            %      % Close the associated figure window. As long as the
            %      % interactiveRigidBodyTree object exists, this actually
            %      % hides the figure rather than deleting it (once the
            %      % object is cleared, that's no longer the case).
            %      close(ax.Parent);
            %
            %      % Make the figure visible again
            %      showFigure(viztree);
            
            obj.VisualizationHelper.showFigure;
        end
    end
    
    %% Get/set methods
    
    methods
        function tree = get.RigidBodyTree(obj)
            %get.RigidBodyTree Get method for obj.RigidBodyTree
            %   The RigidBodyTree property may be defined at construction
            %   as a Name-Value pair. After construction, the property is
            %   read-only.
            
            tree = obj.Backend.RigidBodyTree;
        end
        
        function ikSolver = get.IKSolver(obj)
            %get.IKSolver Get method for obj.IKSolver
            %   The IKSolver property may be defined at construction as a
            %   Name-Value pair. After construction, the property is
            %   read-only.
            
            ikSolver = obj.Backend.IKSolver;
        end
        
        function set.MarkerBodyName(obj, eeName)
            %set.MarkerBodyName Set method for obj.MarkerBodyName
                        
            % Set the end effector name and current transform
            obj.Backend.MarkerBodyName = obj.validateMarkerBodyName(obj.Backend.RigidBodyTree, eeName);
            
            % Attempt to turn on ShowMarker (if the marker is invalid for
            % some reason, it will fail during this step).
            obj.ShowMarker = true;
            
            % If the marker was turned on, update the pose target
            if obj.ShowMarker
                obj.Backend.updatePoseTarget;
            end
        end
        
        function eeName = get.MarkerBodyName(obj)
            %get.MarkerBodyName Get method for obj.MarkerBodyName
            
            eeName = obj.Backend.MarkerBodyName;
        end
        
        function markerPose = get.MarkerPose(obj)
            %get.MarkerPose Get method for obj.MarkerPose
            %   The MarkerPose property is read-only.
            
             markerPose = obj.VisualizationHelper.MarkerPose;
        end
        
        function markerBodyPose = get.MarkerBodyPose(obj)
            %get.MarkerBodyPose Get method for obj.MarkerBodyPose
            %   The MarkerBodyPose property is read-only.
            
            markerBodyPose = obj.Backend.MarkerBodyPose;
        end
        
        function set.MarkerControlMethod(obj, methodStr)
            %set.MarkerControlMethod Set method for obj.MarkerControlMethod
            
            % Process input and turn on the marker
            obj.Backend.MarkerControlMethod = obj.validateMarkerControlMethod(methodStr);
            obj.ShowMarker = true;
        end
        
        function methodStr = get.MarkerControlMethod(obj)
            %get.MarkerControlMethod Get method for obj.MarkerControlMethod
            
            methodStr = obj.Backend.MarkerControlMethod;
        end
        
        function set.Configuration(obj, cfg)
            %set.Configuration Set method for obj.Configuration
            
            %Validate and set internal property
            obj.Backend.Configuration = obj.validateConfiguration(obj.RigidBodyTree, cfg);
            
            % Update the visualization
            obj.updateVisualization
            
            % Update the marker position if it exists
            if obj.ShowMarker
                obj.VisualizationHelper.updateMarker(obj.Backend.MarkerBodyPose);
            end
        end
        
        function cfg = get.Configuration(obj)
            %get.Configuration Get method for obj.Configuration
            
            cfg = obj.Backend.Configuration;
        end
        
        function set.SolverPoseWeights(obj, wts)
            %set.SolverPoseWeights Set method for obj.SolverPoseWeights
            
            % Process input
            obj.Backend.GIKPoseTarget.Weights = obj.validateSolverPoseWeights(wts);
        end
        
        function wts = get.SolverPoseWeights(obj)
            %get.SolverPoseWeights Get method for obj.SolverPoseWeights
            
            wts = obj.Backend.GIKPoseTarget.Weights;
        end
        
        function set.Constraints(obj, ikconstraints)
            %set.Constraints Set method for obj.Constraints
            
            % Set the constraints
            obj.Backend.Constraints = obj.validateIKConstraints(ikconstraints);
            
            % Update the IKSolver to match
            obj.Backend.updateIKConstraints;
            
            % Run the GIK solver to find the updated configuration given
            % the current marker body pose
            obj.Backend.updateIK(obj.Configuration, obj.MarkerBodyPose);
            
            % Update the visualization
            obj.updateVisualization
            
            % Update the marker position if it exists
            if obj.ShowMarker
                obj.VisualizationHelper.updateMarker(obj.Backend.MarkerBodyPose);
            end            
        end
        
        function ikconstraints = get.Constraints(obj)
            %get.Constraints Get method for obj.Constraints
            
            ikconstraints = obj.Backend.Constraints;
        end
        
        function set.MarkerScaleFactor(obj, sf)
            %set.MarkerScaleFactor Set method for obj.MarkerScaleFactor
            
            % Validate input
            obj.Backend.MarkerScaleFactor = obj.validateMarkerScaleFactor(sf);
            
            % Turn on the marker
            obj.ShowMarker = true;
        end
        
        function sf = get.MarkerScaleFactor(obj)
            %get.MarkerScaleFactor Get method for obj.MarkerScaleFactor
            
            sf = obj.Backend.MarkerScaleFactor;
        end
        
        function set.ShowMarker(obj, isOnOff)
            %set.ShowMarker Set method for obj.ShowMarker
            
            % Validate input
            obj.Backend.ShowMarker = obj.validateShowMarker(isOnOff);
            
            % Process flag action
            if isOnOff
                obj.VisualizationHelper.createMarker;
            else
                delete(obj.VisualizationHelper.Marker);
            end
        end
        
        function isOnOff = get.ShowMarker(obj)
            %get.ShowMarker Get method for obj.ShowMarker
            
            isOnOff = obj.Backend.ShowMarker;
        end
        
        function set.StoredConfigurations(obj, scMatrix)
            %set.StoredConfigurations Set method for obj.StoredConfigurations
            
            obj.Backend.StoredConfigurations = obj.validateStoredConfigurations(obj.RigidBodyTree, scMatrix);
        end        
        
        function sc = get.StoredConfigurations(obj)
            %get.StoredConfigurations Get method for obj.StoredConfigurations
            
            sc = obj.Backend.StoredConfigurations;
        end
    end
    
    %% Validation
    
    methods (Access = protected)
        
        function updateVisualization(obj)
            obj.VisualizationHelper.updateAxesConfig;
        end
        
        function [ikSolver, solverPoseWeights, ikConstraintsProp] = initializeIKProperties(obj, tree, parser, ikSolverName, solverPoseWeightsName, ikConstraintsName)
            
            % Extract inputs from input parser
            ikSolverInput = parameterValue(parser, ikSolverName);
            solverPoseWeights = parameterValue(parser, solverPoseWeightsName);
            ikConstraintsProp = parameterValue(parser, ikConstraintsName);
            
            % Validate solver
            if ~isempty(ikSolverInput)
                validateattributes(ikSolverInput, {'generalizedInverseKinematics'}, {'nonempty', 'scalar'}, 'interactiveRigidBodyTree', 'IKSolver');
                
                % Create a new GIK solver from the object but using the
                % tree that matches the rigidBodyTree that will be used in
                % constraints in this object
                ikSolver = generalizedInverseKinematics(...
                    'ConstraintInputs', ikSolverInput.ConstraintInputs, ...
                    'RigidBodyTree', tree, ...
                    'SolverAlgorithm', ikSolverInput.SolverAlgorithm, ...
                    'SolverParameters', ikSolverInput.SolverParameters);
            else
                ikSolver = generalizedInverseKinematics('RigidBodyTree', tree, 'SolverAlgorithm', 'Lev');
                ikSolver.SolverParameters.MaxIterations = 2;
            end
            
            % Validate weights and constraints
            solverPoseWeights = obj.validateSolverPoseWeights(solverPoseWeights);
            ikConstraintsProp = obj.validateIKConstraints(ikConstraintsProp);
            
            % Update the IKSolver to have the correct constraints
            inputConstraintTypes = generateConstraintTypes(ikConstraintsProp);
            ikSolver.ConstraintInputs = [{'pose'} inputConstraintTypes];
        end
        
        function [markerBodyName, markerScaleFactor, showMarker, markerControlMethod] = initializeMarkerProperties(obj, tree, parser, markerBodyNameName, markerScaleFactorName, showMarkerName, markerControlMethodName)
            %initializeMarkerProperties Initialize properties with inter-dependent default parameters
            %   These properties are related to the marker, and have
            %   pre-defined default values. However, depending on the
            %   combination of values provided by the user at construction,
            %   the default value may be changed. For example, by default,
            %   ShowMarker is set to false. However, if the user sets the
            %   MarkerName, then the marker should be shown.
            
            % Extract inputs from input parser
            markerBodyName = parameterValue(parser, markerBodyNameName);
            markerScaleFactor = parameterValue(parser, markerScaleFactorName);
            showMarker = parameterValue(parser, showMarkerName);
            markerControlMethod = parameterValue(parser, markerControlMethodName);
            
            % Record what was provided by the user
            isMarkerNameProvided = ~isempty(markerBodyName);
            isShowMarkerProvided = ~isempty(showMarker);
            isCtrlMethodProvided = ~isempty(markerControlMethod);
            
            % Replace with default values if no input was provided
            if ~isMarkerNameProvided
                markerBodyName = tree.Bodies{end}.Name;
            end
            if ~isShowMarkerProvided
                showMarker = obj.ShowMarkerDefault;
            end
            if ~isCtrlMethodProvided
                markerControlMethod = obj.MarkerControlMethodDefault;
            end
            
            % Validate inputs
            markerBodyName = obj.validateMarkerBodyName(tree, markerBodyName);
            markerScaleFactor = obj.validateMarkerScaleFactor(markerScaleFactor);
            showMarker = obj.validateShowMarker(showMarker);
            markerControlMethod = obj.validateMarkerControlMethod(markerControlMethod);
            
            % Handle special cases
            if isMarkerNameProvided && ~isShowMarkerProvided
                % If marker name is explicitly set but ShowMarker is not
                % set, enable the marker
                showMarker = true;
            elseif isCtrlMethodProvided && ~isShowMarkerProvided
                % If the control method has explicitly been set, and show
                % marker has not been turned off, enable the marker
                showMarker = true;
            end
        end
    end
    
    methods (Static, Access = private)
        
        function constraintTargets = validateIKConstraints(constraintTargets)
            
            if isempty(constraintTargets)
                validateattributes(constraintTargets, {'cell'}, {}, 'interactiveRigidBodyTree', 'Constraints');
            else
                validateattributes(constraintTargets, {'cell'}, {'2d','nrows',1}, 'interactiveRigidBodyTree', 'Constraints');
            end
        end
        
        function solverWeights = validateSolverPoseWeights(solverPoseWeights)
            
            robotics.internal.validation.validateNumericMatrix( ...
                solverPoseWeights, 'interactiveRigidBodyTree', 'SolverPoseWeights', ...
                'numel', 2, 'nonnan', 'finite', 'real', 'nonnegative');
            solverWeights = double(solverPoseWeights(:)');
        end
        
        function markerBodyName = validateMarkerBodyName(tree, bodyNameStr)
            validateattributes(bodyNameStr, {'string', 'char'}, {'scalartext'}, 'interactiveRigidBodyTree', 'MarkerBodyName');
            bid = tree.TreeInternal.findBodyIndexByName(bodyNameStr);
            if bid == -1
                robotics.manip.internal.error('rigidbodytree:BodyNotFound', bodyNameStr);
            end
            markerBodyName = bodyNameStr;
        end
        
        function markerScaleFactor = validateMarkerScaleFactor(scaleFactor)
            validateattributes(scaleFactor, {'numeric'}, {'nonempty', 'scalar', 'real', 'positive', 'finite'}, 'interactiveRigidBodyTree', 'MarkerScaleFactor');
            markerScaleFactor = scaleFactor;
        end
        
        function showMarker = validateShowMarker(smFlag)
            validateattributes(smFlag, {'logical'}, {'nonempty', 'scalar'}, 'interactiveRigidBodyTree', 'MarkerScaleFactor');
            showMarker = smFlag;
        end
        
        function markerControlMethod = validateMarkerControlMethod(methodStr)
            methodStr = validatestring(methodStr, {'JointControl', 'InverseKinematics'}, 'interactiveRigidBodyTree', 'MarkerControlMethod');
            markerControlMethod = methodStr;
        end
        
        function config = validateConfiguration(tree, configVector)
            validateattributes(configVector, {'double'}, {'nonempty', 'vector', 'finite', 'real', 'numel', tree.TreeInternal.VelocityNumber}, 'interactiveRigidBodyTree', 'Configuration');
            config = configVector(:);
        end
        
        function storedConfigs = validateStoredConfigurations(tree, scMatrix)
            if isempty(scMatrix)
                numRowsConstraint = {};
            else
                numRowsConstraint = {'nrows', tree.TreeInternal.VelocityNumber};
            end
                validateattributes(scMatrix, {'double'}, [{'2d', 'finite', 'real'} numRowsConstraint], 'interactiveRigidBodyTree', 'StoredConfigurations');
            storedConfigs = scMatrix;
        end
    end
end

function cTypes = generateConstraintTypes(constraintTargets)
cTypes = cell(1, numel(constraintTargets));
for i = 1:numel(cTypes)
    switch class(constraintTargets{i})
        case 'constraintAiming'
            cTypes{i} = 'aiming';
        case 'constraintCartesianBounds'
            cTypes{i} = 'cartesian';
        case 'constraintJointBounds'
            cTypes{i} = 'joint';
        case 'constraintOrientationTarget'
            cTypes{i} = 'orientation';
        case 'constraintPositionTarget'
            cTypes{i} = 'position';
        case 'constraintPoseTarget'
            cTypes{i} = 'pose';
        case 'constraintRevoluteJoint'
            cTypes{i} = 'revolutejoint';
        case 'constraintPrismaticJoint'
            cTypes{i} = 'prismaticjoint';
        case 'constraintFixedJoint'
            cTypes{i} = 'fixedjoint';
    end
end
end

