classdef SolverModel < robotics.ikdesigner.internal.model.Model
%This class if for internal use only and may be removed in a future release

%SolverModel Model that contains IK solver and solution data

%   Copyright 2021-2022 The MathWorks, Inc.

    events

        %IKSolutionUpdated Event to indicate that a new IK solution has been computed
        IKSolutionUpdated

        %EEBodyChanged Event to indicate that the end effector body has been reassigned
        EEBodyChanged

        %IKPropertiesChanged Event to indicate that the IK Solver properties have changed
        IKPropertiesChanged

        %IKConstraintStateUpdated Event to indicate that the state of the IK constraints have changed
        IKConstraintStateUpdated

        %MarkerPoseTargetDisabled Event to indicate that marker pose target has been disabled
        MarkerPoseTargetDisabled

        %MarkerPoseTargetEnabled Event to indicate that marker pose target has been enabled
        MarkerPoseTargetEnabled

        %NewConstraintDraftSent Event to indicate that a new constraint draft has been sent
        NewConstraintDraftSent

        %EditConstraintDraftSent Event to indicate that an edit constraint draft has been sent
        EditConstraintDraftSent

        %InstructEnableConstraintButton Event to indicate that the constraint button should be enabled
        %   This event is sent when the user should be able to click on an
        %   "edit constraint" button. Typically this happens when an
        %   eligible constraint is selected.
        InstructEnableConstraintEditing

        %InstructDisableConstraintEditing Event to indicate that the constraint button should be disabled
        %   This event is sent when the user should not be able to click on an
        %   "edit constraint" button. This can happen if for example the
        %   user is not selecting an eligible constraint.
        InstructDisableConstraintEditing

        %InstructUpdateSolverTab Event to update the solver tab in the toolstrip with new solver settings
        InstructUpdateSolverTab

        %RequestUIAlert Create a ui alert for a message
        RequestUIAlert
    end

    properties (SetAccess = ?matlab.unittest.TestCase)

        %IKSolver GIK object
        IKSolver

        %LastSolution Most recent solution produced by GIK solver
        LastSolution

        %LastSolution Solution info output associated with the most recent solution
        LastSolutionInfo

        %LastSolutionConstraintKeys Keys for constraints included in the most recent solution
        LastSolutionConstraintKeys

        %ConstraintsMap Map of GIK constraints
        ConstraintsMap
    end

    properties (Access = ?matlab.unittest.TestCase)

        %EEBodyName End effector body name used for GIK Pose target
        EEBodyName

        %SelectedConstraintKey Key associated with the node currently selected in the constraints browser
        SelectedConstraintKey

        %SharedModelState Handle to a model containing core scene states
        SharedModelState
    end

    properties (Dependent)

        %SceneObjectsMap Map of objects in the scene, actually stored in the shared model state
        SceneObjectsMap

        %RigidBodyKeysMap Map to relate body names to their keys, actually stored in the shared model state
        RigidBodyKeysMap
    end

    properties (Constant)
        %MarkerPoseTargetKey Key associated with the marker pose constraint
        %   The marker pose constraint is stored in the constraints map,
        %   but it used a reserved key that cannot overlap with a UUID
        %   value. Since this constraint cannot be removed, the value is
        %   persistent. This allows the key to be easily accessed from a
        %   constraints map in the model or views.
        MarkerPoseTargetKey = robotics.ikdesigner.internal.constants.Data.MARKERPOSETARGETKEY

        %DefaultSolutionInfo A dummy solver result to be used for an assigned solution
        DefaultSolutionInfo = struct( ...
            'Iterations', 1, ...
            'NumRandomRestarts', 0, ...
            'ConstraintViolations', struct('Type', {'pose'}, 'Violation', {0}), ...
            'ExitFlag', 1, ...
            'Status', 'Best available');
    end

    properties (Constant)
        MARKERPOSETARGETNAME = string(message('robotics:ikdesigner:constrainttabviews:MarkerPoseTargetName'))
        CONSTRAINTNOTLOADEDHEADER = string(message('robotics:ikdesigner:dataimportexport:ConstraintsNotLoadedHeader'))
    end

    properties (Dependent, SetAccess = private)
        %LastSolutionEEPose The end effector pose that corresponds to the most recent solution (read-only)
        LastSolutionEEPose

        %LastSolutionState
        LastSolutionState

        %GIKPoseTarget Pose target (homogeneous transform) associated with the marker pose constraint
        GIKPoseTarget
    end

    methods
        function obj = SolverModel(sharedModelState)
        %SolverModel Constructor

        % Assign property defaults. The defaults used here just assign
        % data type; the object's lifetime is persistent over the
        % course of the app, but the properties are assigned in the
        % setup() method, which is called every time a session is
        % loaded.
            obj.SharedModelState = sharedModelState;
            obj.EEBodyName = string.empty;
            obj.ConstraintsMap = containers.Map.empty;
            obj.LastSolution = [];
            obj.SelectedConstraintKey = string.empty;

            % The inverse kinematics solver will be an object of type
            % "generalizedInverseKinematics", which has no meaningful empty
            % constructor without an associated rigid body tree
            obj.IKSolver = [];

        end

        function setup(obj, modelData)
        %setup Reset data members and set up with new incoming data

            if nargin > 1
                % If model data is provided, set up the model from existing
                % data
                obj.IKSolver = modelData.IKSolver;
                obj.EEBodyName = modelData.EEBodyName;
                obj.ConstraintsMap = modelData.ConstraintsMap;
                obj.LastSolution = modelData.LastSolution;
                obj.LastSolutionInfo = modelData.LastSolutionInfo;
                obj.LastSolutionConstraintKeys = modelData.LastSolutionConstraintKeys;
            else
                % Set up the model with new data
                robot = obj.SharedModelState.RigidBodyTree;
                obj.IKSolver = generalizedInverseKinematics('RigidBodyTree', robot);
                obj.IKSolver.SolverAlgorithm = robotics.ikdesigner.internal.constants.Data.DEFAULTSOLVERALGORITHM;
                obj.IKSolver.SolverParameters = robotics.ikdesigner.internal.constants.Data.DEFAULTSOLVERPARAMS;
                obj.EEBodyName = robot.BodyNames{end};

                % Load the constraint map.
                obj.setupConstraintsMap();

                % Initially, robot should be in its home configuration
                obj.LastSolution = robot.homeConfiguration;
                obj.LastSolutionInfo = obj.DefaultSolutionInfo;
                obj.LastSolutionConstraintKeys = obj.MarkerPoseTargetKey;
            end
        end

        function initialize(obj)
        %initialize Initialize a new session

        % Create the marker pose target and add it to the constraints
        % map with the appropriate key
            baseBodyName = obj.SharedModelState.RigidBodyTree.BaseName;
            markerPoseTarget = constraintPoseTarget(obj.EEBodyName, "ReferenceBody", baseBodyName);
            markerPoseTarget.TargetTransform = getTransform(obj.SharedModelState.RigidBodyTree, obj.SharedModelState.CurrentConfig, obj.EEBodyName);
            obj.ConstraintsMap(obj.MarkerPoseTargetKey) = obj.populateMapEntryStruct(obj.MARKERPOSETARGETNAME, ...
                                                                                     robotics.ikdesigner.internal.toolstrip.ConstraintType.Pose, markerPoseTarget, [], false);

            % Update the IK constraint types, which is set on the solver
            obj.updateIKSolverConstraintTypes;
        end

        function addCustomConstraints(obj, customConstraintEvent)
        %addCustomConstraints Add constraints from an external source
        %   This method accepts an event that provides constraints and
        %   their names from an external source. The method validates
        %   the constraints, adds them to the app, and then notifies
        %   the controller of the update. Any invalid constraints are
        %   not added, but for these, a UI alert is instead displayed
        %   indicating the incompatibility to the user.

        % Initialize outputs for error handling. Note the string cannot
        % be empty since the intent is to concatenate non-empty values
        % to it.
            allErrorsString = "";

            constraintArray = customConstraintEvent.Data;
            constraintNamesArray = customConstraintEvent.DataNames;
            for i = 1:numel(constraintArray)

                % Verify that the constraint is compatible with the current
                % solver / robot configuration. If not, an empty key is
                % returned.
                constraintToAdd = constraintArray{i}.copy;
                constraintName = string(constraintNamesArray{i});
                [isValidConstraint, constraintTypeEnum, validationErrorStrings] = obj.validateConstraintInput(constraintToAdd);

                if isValidConstraint
                    constraintKey = matlab.lang.internal.uuid;
                    constraintStateDetails = [];
                    isDisabled = false;
                    obj.ConstraintsMap(constraintKey) = obj.populateMapEntryStruct(constraintName, constraintTypeEnum, constraintToAdd, constraintStateDetails, isDisabled);
                    obj.notifyConstraintsMapChange(constraintKey);
                else
                    constraintErrorString = obj.createConstraintImportErrorString(constraintName, validationErrorStrings);
                    allErrorsString = allErrorsString + newline + constraintErrorString;
                end
            end

            if ~strcmp(allErrorsString, "")
                % Throw an error that indicates that not all the
                % constraints were successfully imported
                errorHeader = obj.CONSTRAINTNOTLOADEDHEADER + newline;
                obj.notifyUIAlert(errorHeader + allErrorsString);
            end
        end

        function notifyUIAlert(obj, messageString)
        %notifyUIAlert Notify controller that the app window needs to issue a UI alert

            messageEvt = robotics.ikdesigner.internal.event.MessageEvent(messageString);
            notify(obj, 'RequestUIAlert', messageEvt);
        end

        function updateIKConstraintsMap(obj, constraintsMapEvent)
        %updateIKConstraintsMap Add or edit values in the constraints map

        % Since the key is specified, update only the items in the
        % given key
            keyToChange = constraintsMapEvent.Key;
            obj.ConstraintsMap(keyToChange) = obj.populateMapEntryFromEvent(constraintsMapEvent);
            obj.notifyConstraintsMapChange(keyToChange);
        end

        function removeConstraintsFromSolverModel(obj, deleteEvent)
        %removeObjectFromSceneModel Delete constraints from the constraints map

        % Remove the object from the scene
            obj.ConstraintsMap.remove(deleteEvent.Key);
            obj.notifyConstraintsMapChange(deleteEvent.Key);

        end

        function notifyConstraintsMapChange(obj, changedKey)
        %notifyConstraintsMapChange Take action on changes to constraints map
        %   This method propagates changes to the constraints map by
        %   updating the types in the solver and notifying the
        %   controllers of these changes.

        % Update the constraints in the solver
            obj.updateIKSolverConstraintTypes;

            % Notify the views that the set of constraints have changed
            obj.notifyConstraintsValuesChanged(changedKey);

            % Notify the controller that properties have changed
            notify(obj, "IKPropertiesChanged");
        end

        function updateIKSolverConstraintTypes(obj)
        %updateIKSolverConstraintTypes Update GIK constraint types
        %   The solver stores the type of constraints in the constraint
        %   inputs property. When the solver is called, the constraints
        %   passed to the solver must match the types specified in the
        %   properties. This step generates those properties so that
        %   they will match.

            inputConstraintTypes = obj.generateConstraintTypes(obj.getConstraintsArray);
            obj.IKSolver.release;
            obj.IKSolver.ConstraintInputs = inputConstraintTypes;
        end

        function toggleConstraintEnable(obj, constraintStateEvent)
        %toggleConstraintEnable Switch the enabled/disable state of a constraint
        %   This method updates the state of the constraints. It turns
        %   on the set of constraints with keys that are marked to
        %   enable, and disables the constraints corresponding to keys
        %   that are marked to be disabled.

        % Extract the list of constraint keys to enable and disable
        % from the event data, then iterate over the values and perform
        % the associated action.
            keysToEnable = constraintStateEvent.ConstraintKeysToEnable;
            keysToDisable = constraintStateEvent.ConstraintKeysToDisable;

            for i = 1:numel(keysToEnable)
                constraintEntry = obj.ConstraintsMap(keysToEnable(i));
                constraintEntry.State = robotics.ikdesigner.internal.model.ConstraintState.Unset;
                obj.ConstraintsMap(keysToEnable{i}) = constraintEntry;
            end

            for j = 1:numel(keysToDisable)
                constraintEntry = obj.ConstraintsMap(keysToDisable(j));
                constraintEntry.State = robotics.ikdesigner.internal.model.ConstraintState.Disabled;
                obj.ConstraintsMap(keysToDisable{j}) = constraintEntry;
            end

            % If the marker pose target is disabled, turn off its
            % associated visuals. When it is turned back on, reset the
            % visual to match the pose of the body at its current location
            if any(keysToDisable == obj.MarkerPoseTargetKey)
                notify(obj, "MarkerPoseTargetDisabled");
            end
            if any(keysToEnable == obj.MarkerPoseTargetKey)
                obj.updateEEPoseBody(obj.EEBodyName);
                notify(obj, "MarkerPoseTargetEnabled");
            end

            % Update the constraints in the solver
            obj.updateIKSolverConstraintTypes;

            % Notify the controller that properties have changed, which
            % calls the solver refresh
            notify(obj, "IKPropertiesChanged");
        end

        function updateMarkerPoseTarget(obj, targetPose)
        %updateMarkerPoseTarget Update the marker constraint's pose target
        %   This method updates the value of the target pose. It only
        %   updates the value; it does not also call the IK mechanism.

            obj.GIKPoseTarget.TargetTransform = targetPose;
        end

        function updateIK(obj, currConfig)
        %updateIK Update built-in position and orientation targets
        %   Set the target pose of the primary GIK PoseTarget
        %   constraint object to a specified pose using the current
        %   configuration as an initial guess.
        % Get constraints and matching keys
            [constraintsArray, keys] = obj.getConstraintsArray;

            % Suppress joint validation
            warning('off', 'robotics:robotmanip:rigidbodytree:ConfigJointLimitsViolationAutoAdjusted');

            % Solve GIK with applied constraints and update pose
            [newConfig, info] = obj.IKSolver(currConfig, constraintsArray{:});
            obj.LastSolution = newConfig;
            obj.LastSolutionInfo = info;
            obj.LastSolutionConstraintKeys = keys;

            % Unsuppress user-facing warning
            warning('on', 'robotics:robotmanip:rigidbodytree:ConfigJointLimitsViolationAutoAdjusted');

            % Update the constraint state to indicate whether or not
            % constraints are achieved in this solution
            obj.updateConstraintState(keys, info.ConstraintViolations);

            % Update the solver config and details display
            obj.notifySolutionUpdated();
        end

        function applyPredefinedState(obj, config, ikState)
        %applyPredefinedState Override the solver using a defined state

        % Get the solver info. If it has been reset by a constraint
        % change, replace with cleared data
            info = ikState.Info;
            if isempty(info)
                constraintKeys = string.empty;
                constraintViolations = [];
            else
                constraintKeys = ikState.ConstraintKeys;
                constraintViolations = info.ConstraintViolations;
            end

            obj.LastSolution = config;
            obj.LastSolutionInfo = info;

            obj.resetConstraintState;
            obj.updateConstraintState(constraintKeys, constraintViolations);
            obj.manuallyApplyMarkerPoseTarget();

            obj.notifySolutionUpdated();
        end

        function applyJointConfiguration(obj, config)
        %applyJointConfiguration Override the solver using a defined joint configuration
        %   This method only updates the joint configuration. It does not
        %   modify the solver or state parameters. Those properties are set
        %   in obj.applyPredefinedState().

            % Reset the constraint state since it is no longer valid at
            % this configuration. Then update the constraint views.
            obj.resetConstraintState;
            obj.updateConstraintState(string.empty, []);
    
            obj.LastSolution = config;
            obj.updateMarkerPoseFromConfig(config);
            obj.notifySolutionUpdated();

        end

        function updateIKSettings(obj, solverSettingsContainer)
        %updateIKSettings Change the solver algorithm or parameters
        %   Update the solver settings given an event or object
        %   containing the new settings.

            obj.IKSolver.release();
            obj.IKSolver.SolverAlgorithm = solverSettingsContainer.SolverAlgorithm;
            obj.IKSolver.SolverParameters = solverSettingsContainer.SolverParameters;

            notify(obj, "IKPropertiesChanged");
            obj.notifyModelBecameDirty();
        end

        function replaceSolverParameters(obj, ikSolver)
        %replaceSolverParameters Substitute the IK solver settings with settings from another solver
        %   This method replaces the current solver algorithm and
        %   settings, which has the effect of swapping out a new
        %   solver. The constraints are not inherited as these are
        %   passed separately.

            obj.updateIKSettings(ikSolver);

            % Updating the settings just updates the model (since all
            % changes are already incurred on the view), but when the
            % solver is replaced, it is necessary to also update the solver
            % tab display.
            notify(obj, "InstructUpdateSolverTab");
        end

        function updateSelectedConstraint(obj, event)
        %updateSelectedConstraint Update the selected constraint
        %   This method is used by controllers to update the stored
        %   value of SelectedConstraintKey. The method sets the key,
        %   then based on the value, indicates whether or not the
        %   associated buttons in the views should be clickable or not.

            selectedKey = event.ConstraintKey;
            if isempty(selectedKey)

                % If the selected key is empty, the associated node is not
                % associated with a constraint and the button should be
                % disabled. It is necessary to check this first to avoid
                % errors tied to the comparator method since [] is a
                % double.
                obj.SelectedConstraintKey = [];
                notify(obj, "InstructDisableConstraintEditing");
            elseif obj.ConstraintsMap.isKey(selectedKey)

                % If the selected key is found in the constraints map, it
                % corresponds to an eligible constraint and edit should be
                % enabled.
                obj.SelectedConstraintKey = selectedKey;
                notify(obj, "InstructEnableConstraintEditing");
            else
                obj.SelectedConstraintKey = [];
                notify(obj, "InstructDisableConstraintEditing");
            end
        end

        function updateEEPoseBody(obj, markerBodyName)
        %updateEEPoseBody Update the end effector body name

            obj.EEBodyName = markerBodyName;
            obj.GIKPoseTarget.EndEffector = markerBodyName;

            notify(obj, "EEBodyChanged");
            obj.notifyModelBecameDirty();

            obj.manuallyApplyMarkerPoseTarget();
        end

        function [constraintKey, constraintName] = getNewConstraintData(obj)
        %getNewConstraintData Get a key and default name for a new constraint
        %   To create a new constraint, a key and name are required.
        %   The name should avoid conflict with existing constraint
        %   names.

        % Generate a new key
            constraintKey = matlab.lang.internal.uuid;

            % Choose a name that isn't already in use
            nameIdx = 1;
            constraintName = "Constraint1"; %TODO: I18N
            existingNames = cellfun(@(x)(x.Name), obj.ConstraintsMap.values, "UniformOutput",true);
            while any(strcmp(constraintName, existingNames))
                constraintName = sprintf("Constraint%i", nameIdx);
                nameIdx = nameIdx+1;
            end
        end

        function [constraintKey, constraintName, constraintType, constraintData] = getActiveConstraintData(obj)
        %getActiveConstraintData Get a key and default name for the currently selected constraint
        %   Get the core items needed to display details from a the
        %   selected constraint in a view. This includes the key, name,
        %   type, and data struct, which translates constraint
        %   properties to view-friendly contents (e.g. body key instead
        %   of body name).

        % Get the a draft constraint name from the current count &
        % generate a new key
            constraintKey = obj.SelectedConstraintKey;
            [constraintName, constraintType, constraintData] = obj.getConstraintData(obj, constraintKey);
        end

        function [constraintKey, constraintName, constraintType, constraintData] = getMarkerPoseConstraintData(obj)
        %getMarkerPoseConstraintData Get a key and default name for the marker pose constraint

            constraintKey = obj.MarkerPoseTargetKey;
            [constraintName, constraintType, constraintData] = obj.getConstraintData(obj, constraintKey);
        end
    end

    % Get/Set methods
    methods

        function map = get.SceneObjectsMap(obj)
        % Getter for SceneObjectsMap

            map = obj.SharedModelState.SceneObjectsMap;
        end

        function map = get.RigidBodyKeysMap(obj)
        % Getter for RigidBodyKeysMap

            map = obj.SharedModelState.RigidBodyKeysMap;
        end

        function eePose = get.LastSolutionEEPose(obj)
        % Getter for LastSolutionEEPose

            eePose = getTransform(obj.IKSolver.RigidBodyTree, obj.LastSolution, obj.EEBodyName);
        end

        function state = get.LastSolutionState(obj)
        % Getter for LastSolutionState

            state = robotics.ikdesigner.internal.model.IKState(obj.LastSolutionInfo, obj.LastSolutionConstraintKeys);
        end

        function [solver, constraintsArray, constraintNamesArray] = exportSolverAndConstraints(obj)
        %exportSolverAndConstraints Export solver and constraints
        %   Export the solver with all possible constraints (not just
        %   the currently active ones). This method makes copies of the
        %   solver and constraints so that they are independent of the
        %   ones uses by the app (since these are handle objects).
        %   Additionally, the constraints are returned in an order such
        %   that the marker pose target is the first constraint.

            constraintEntriesArray = obj.ConstraintsMap.values;
            constraintsArray = cellfun(@(x)(x.Constraint.copy), constraintEntriesArray, 'UniformOutput', false);
            constraintNamesArray = cellfun(@(x)(x.Name), constraintEntriesArray, 'UniformOutput', false);

            % Change the order so the marker pose target will be first. The
            % others may still be out of order relative to the browser due
            % to the fact that the UUID is not necessarily incremental
            constraintsArray = fliplr(constraintsArray);
            constraintNamesArray = fliplr(constraintNamesArray);

            % Copy the solver (so a different handle is exported) and
            % update the constraint inputs so they exported constraint list
            solver = obj.IKSolver.clone();
            solver.release();
            inputConstraintTypes = obj.generateConstraintTypes(constraintsArray);
            solver.ConstraintInputs = inputConstraintTypes;
        end

        function poseTarget = get.GIKPoseTarget(obj)
        %get.GIKPoseTarget Get the marker pose constraint pose target
        %   The pose target for the marker pose constraint is a
        %   property of the constraint, which is stored in the
        %   containers.Map object in the ConstraintsMap property.

            poseTarget = obj.ConstraintsMap(obj.MarkerPoseTargetKey).Constraint;
        end

        function set.GIKPoseTarget(obj, poseTarget)
        %set.GIKPoseTarget Set the marker pose constraint pose target
        %   The pose target for the marker pose constraint is a
        %   property of the constraint, which is stored in the
        %   containers.Map object in the ConstraintsMap property.
        %   However, only one level of indexing is supported by a
        %   containers.Map, so this method ensures that the value can
        %   be easily updated without having to execute the following
        %   three lines each time a field on the marker pose target
        %   constraint is changed.

            poseTargetMapEntry = obj.ConstraintsMap(obj.MarkerPoseTargetKey);
            poseTargetMapEntry.Constraint = poseTarget;
            obj.ConstraintsMap(obj.MarkerPoseTargetKey) = poseTargetMapEntry;

        end
    end

    methods (Access = private)

        function setupConstraintsMap(obj)
        %setupConstraintsMap Set up the constraints map with data

            obj.ConstraintsMap = containers.Map.empty;
        end

        function entryStruct = populateMapEntryFromEvent(obj, constraintsMapEvent)
        %populateMapEntryFromEvent Populate constraints map from event
        %   This method creates a constraints map value given a
        %   constraint event (e.g. when a new constraint is added or an
        %   existing one is edited).

            constraintName = constraintsMapEvent.Name;
            constraintType = constraintsMapEvent.Type;

            constraintData = constraintsMapEvent.Data;
            switch constraintType
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Aiming
                eeName = obj.SceneObjectsMap(constraintData.EEBodyKey).Name;
                constraint = constraintAiming(eeName, ...
                                              'ReferenceBody', obj.SceneObjectsMap(constraintData.RefBodyKey).Name, ...
                                              'TargetPoint', constraintData.TargetPoint, ...
                                              'AngularTolerance', constraintData.AngularTolerance, ...
                                              'Weights', constraintData.Weights ...
                                             );
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Cartesian
                eeName = obj.SceneObjectsMap(constraintData.EEBodyKey).Name;
                constraint = constraintCartesianBounds(eeName, ...
                                                       'ReferenceBody', obj.SceneObjectsMap(constraintData.RefBodyKey).Name, ...
                                                       'TargetTransform', constraintData.TargetTransform, ...
                                                       'Bounds', constraintData.Bounds, ...
                                                       'Weights', constraintData.Weights ...
                                                      );
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.JointBounds
                constraint = constraintJointBounds(obj.IKSolver.RigidBodyTree, ...
                                                   'Bounds', constraintData.Bounds, ...
                                                   'Weights', constraintData.Weights ...
                                                  );
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Pose
                eeName = obj.SceneObjectsMap(constraintData.EEBodyKey).Name;
                constraint = constraintPoseTarget(eeName, ...
                                                  'ReferenceBody', obj.SceneObjectsMap(constraintData.RefBodyKey).Name, ...
                                                  'TargetTransform', constraintData.TargetTransform, ...
                                                  'OrientationTolerance', constraintData.OrientationTolerance, ...
                                                  'PositionTolerance', constraintData.PositionTolerance, ...
                                                  'Weights', constraintData.Weights ...
                                                 );
            end

            % New constraints are unset by default, while other constraints
            % take on the enable/disable value that was already in place
            constraintKey = constraintsMapEvent.Key;
            if ~obj.ConstraintsMap.isKey(constraintKey)
                isDisabled = false;
            else
                isDisabled = (obj.ConstraintsMap(constraintKey).State == robotics.ikdesigner.internal.model.ConstraintState.Disabled);
            end

            % The state details must be empty since they can only be set
            % locally in the model (where the solver is called)
            constraintStateDetails = [];

            entryStruct = obj.populateMapEntryStruct(constraintName, constraintType, constraint, constraintStateDetails, isDisabled);
        end

        function [enabledConstraints, enabledConstraintKeys] = getConstraintsArray(obj)
        %getConstraintsArray Get the cell array of constraints to be passed to each IK solver
        %   When the IK solver is called, it takes as input the cell
        %   array of constraint objects. This cell array must include
        %   only the enabled (active) constraints. Additionally, since
        %   the analysis in the solution details is based on the order
        %   of the constraint input, it is necessary to also return the
        %   keys in the same order, so that the details can be
        %   associated with the correct constraints.

        % Get all the map values and keys. When extracted this way, the
        % indices match.
            allConstraintsMapKeys = obj.ConstraintsMap.keys;
            allConstraintsMapEntries = obj.ConstraintsMap.values;

            % Filter out the set of map entries & keys corresponding to constraints that are enabled
            isConstraintEnabled = cellfun(@(x)(robotics.ikdesigner.internal.model.ConstraintState.checkIfEnabled(x.State)), allConstraintsMapEntries, 'UniformOutput', true);
            enabledConstraintMapEntries = allConstraintsMapEntries(isConstraintEnabled);
            enabledConstraintKeys = allConstraintsMapKeys(isConstraintEnabled);

            % Get the constraints from these entries (which are structures)
            enabledConstraints = cellfun(@(x)(x.Constraint), enabledConstraintMapEntries, 'UniformOutput', false);
        end

        function updateConstraintState(obj, constraintKeys, constraintViolations)
        %updateConstraintState Update the state of individual constraints
        %   Given two N-element cell arrays, one of constraint keys,
        %   and one of constraint violations, this method updates the
        %   state of the constraints with those keys. The constraint
        %   violations are the raw output of the solver that indicate
        %   how closely a constraint is to being achieved. This method
        %   translates that into boolean PASS/FAIL indicators of
        %   whether or not the constraint was met, updates the state in
        %   the constraints map, and then notifies the controller that
        %   some constraint states have been updated.

        % Iterate over the provided keys an select the map entry for
        % each key
            for i = 1:numel(constraintKeys)
                key = constraintKeys{i};
                mapEntry = obj.ConstraintsMap(key);

                % Store the raw violation data as state details, then
                % translate this to boolean pass/fail
                constraintWeights = mapEntry.Constraint.Weights;
                mapEntry.StateDetails = constraintViolations(i).Violation.*constraintWeights;
                if obj.isConstraintMet(mapEntry.StateDetails)
                    mapEntry.State = robotics.ikdesigner.internal.model.ConstraintState.Pass;
                else
                    mapEntry.State = robotics.ikdesigner.internal.model.ConstraintState.Fail;
                end

                % Update the associated entry in the constraints map
                obj.ConstraintsMap(key) = mapEntry;
            end

            % Notify the views that the constraint state has changed
            obj.notifyConstraintsStateChanged(constraintKeys);

        end

        function resetConstraintState(obj)
        %resetConstraintState Reset constraints for the current configuration to their default (unset) values

            [~, keys] = obj.getConstraintsArray;
            for i = 1:numel(keys)
                key = keys{i};
                mapEntry = obj.ConstraintsMap(key);
                mapEntry.State = robotics.ikdesigner.internal.model.ConstraintState.Unset;
                obj.ConstraintsMap(key) = mapEntry;
            end
        end

        function dataStruct = populateConstraintDataStruct(obj, constraintKey)
        %populateConstraintDataStruct Populate the data struct in the constraint event with info from the constraint
        %   In the events that are sent to and received from the views,
        %   the data associated with the constraint is stored in
        %   structures. The data structure is an exact copy of the
        %   user-facing classes in the constraint itself, with two
        %   major exceptions: for constraints that have a reference
        %   body and end effector body, the struct contains the body
        %   keys instead of the body names.

        %TODO: Replace the structures with value classes (g2572905)

            constraintEntry = obj.ConstraintsMap(constraintKey);
            constraintType = constraintEntry.Type;
            constraint = constraintEntry.Constraint;

            % Populate data
            dataStruct = struct();
            switch constraintType
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Aiming

                dataStruct.EEBodyKey = obj.RigidBodyKeysMap(constraint.EndEffector);
                dataStruct.RefBodyKey = obj.RigidBodyKeysMap(constraint.ReferenceBody);
                dataStruct.TargetPoint = constraint.TargetPoint;
                dataStruct.AngularTolerance = constraint.AngularTolerance;
                dataStruct.Weights = constraint.Weights;

              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Cartesian

                dataStruct.EEBodyKey = obj.RigidBodyKeysMap(constraint.EndEffector);
                dataStruct.RefBodyKey = obj.RigidBodyKeysMap(constraint.ReferenceBody);
                dataStruct.TargetTransform = constraint.TargetTransform;
                dataStruct.Bounds = constraint.Bounds;
                dataStruct.Weights = constraint.Weights;

              case robotics.ikdesigner.internal.toolstrip.ConstraintType.JointBounds

                dataStruct.Bounds = constraint.Bounds;
                dataStruct.Weights = constraint.Weights;

              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Pose

                dataStruct.EEBodyKey = obj.RigidBodyKeysMap(constraint.EndEffector);
                dataStruct.RefBodyKey = obj.RigidBodyKeysMap(constraint.ReferenceBody);
                dataStruct.TargetTransform = constraint.TargetTransform;
                dataStruct.OrientationTolerance = constraint.OrientationTolerance;
                dataStruct.PositionTolerance = constraint.PositionTolerance;
                dataStruct.Weights = constraint.Weights;
            end

        end

        function manuallyApplyMarkerPoseTarget(obj)
            %manuallyApplyMarkerPoseTarget Set the marker pose constraint to a value and indicate to views that is met
            %   The marker pose constraint should always match the marker
            %   pose in the views. Therefore, when the marker is moved to a
            %   pose where it is automatically at the target (e.g. when the
            %   robot is snapped to a previous position, or when the marker
            %   is moved to a new body), it is necessary to update the pose
            %   target values without also calling IK. This method updates
            %   the pose target and notifies the views to tell the user
            %   that the constraint is met. 
            %
            %   Note that this can cause a small point of possible confusion: since the IK
            %   state is unchanged (as it has not been called) previous
            %   values of the IK state details may differ from the
            %   currently perceived behavior. This is intended behavior

            obj.updateMarkerPoseFromConfig(obj.SharedModelState.CurrentConfig);

            markerPoseMetConstraintViolations = struct('Violation', [0 0]);
            updateConstraintState(obj, obj.MarkerPoseTargetKey, markerPoseMetConstraintViolations);

        end

        function updateMarkerPoseFromConfig(obj, config)
            %updateMarkerPoseFromConfig Set the marker pose target to be reflective of a given configuration

            markerBodyName = obj.GIKPoseTarget.EndEffector;
            currentMarkerPoseTarget = obj.SharedModelState.RigidBodyTree.getTransform(config, markerBodyName);
            obj.updateMarkerPoseTarget(currentMarkerPoseTarget);

        end

        function notifySolutionUpdated(obj)
        %notifySolutionUpdated Notify the views that the solution details have changed
        %   Each time a solution is computed, both the solution and the
        %   associated details change. These details are passed to the
        %   views, where they may be displayed to the user as part of
        %   optional advanced displays.

        % Notify that the solver has been updated. The info is
        % available via the public LastSolutionInfo property on this
        % model object.
            notify(obj, "IKSolutionUpdated");
            obj.notifyModelBecameDirty();

        end

        function notifyConstraintsValuesChanged(obj, changedKey)
        %notifyConstraintsValuesChanged Notify the controller that the constraints map values have changed
        %   This indicates that a constraint has been edited, added, or
        %   removed from the model constraints map.

            updateType = robotics.ikdesigner.internal.model.ConstraintUpdate.MapValuesChange;
            constraintsStateEvent = robotics.ikdesigner.internal.event.ConstraintStateData(updateType, obj.ConstraintsMap, changedKey);
            notify(obj, "IKConstraintStateUpdated", constraintsStateEvent);
            obj.notifyModelBecameDirty();

        end

        function notifyConstraintsStateChanged(obj, changedKeys)
        %notifyConstraintsStateChanged Notify the controller that the state of the stored constraints in the constraints map have changed
        %   This indicates that the state of at least one of the stored
        %   constraint has been changed.

            updateType = robotics.ikdesigner.internal.model.ConstraintUpdate.StateChange;
            constraintsStateEvent = robotics.ikdesigner.internal.event.ConstraintStateData(updateType, obj.ConstraintsMap, changedKeys);
            notify(obj, "IKConstraintStateUpdated", constraintsStateEvent);
            obj.notifyModelBecameDirty();
        end

        function [isValid, constraintTypeEnum, errorStrings] = validateConstraintInput(obj, constraintInput)
        %validateConstraintInput Check that imported constraint properties are compatible with the solver
        %   The imported constraints are taken from verified constraint
        %   objects, so their properties have already been validated
        %   for consistency within the constraint framework. However,
        %   it is still necessary to verify that the properties
        %   associated with the solver's rigid body tree object can be
        %   satisfied. Therefore this method checks that the properties
        %   are compatible with the current tree prior to completing
        %   import. The method returns a flag indicating the validity
        %   as well as constraint types and any associated error
        %   strings, if applicable.

            [~, typeEnumCell] = obj.generateConstraintTypes({constraintInput});
            constraintTypeEnum = typeEnumCell{1};

            % Check if the constraint contains these properties, and if so,
            % verify that they are compatible with the tree. If not, return
            % an error string indicating the compatibility issue.
            refBodyError = obj.validateTreeContainsBody("ReferenceBody", obj.SharedModelState.RigidBodyTree, constraintInput);
            eeBodyError = obj.validateTreeContainsBody("EndEffector", obj.SharedModelState.RigidBodyTree, constraintInput);

            errorStrings = [refBodyError eeBodyError];

            % The joint bounds constraint has some properties whose
            % dimension is constraint by the number of non-fixed joints in
            % the associated rigid body tree object
            if constraintTypeEnum == robotics.ikdesigner.internal.toolstrip.ConstraintType.JointBounds
                expJointBoundsSize = size(obj.SharedModelState.RigidBodyTree.TreeInternal.JointPositionLimits);
                expWeightsSize = [1 expJointBoundsSize(1)];
                boundsError = obj.validatePropertyDimensions("Bounds", expJointBoundsSize, constraintInput);
                weightsError = obj.validatePropertyDimensions("Weights", expWeightsSize, constraintInput);
                errorStrings = [errorStrings boundsError weightsError];
            end

            % Output is valid if no error strings have been created
            isValid = isempty(errorStrings);
        end
    end

    methods (Static)

        function entryStruct = populateMapEntryStruct(constraintName, constraintType, constraint, constraintStateDetails, isDisabled)
        %populateMapEntryStruct Populate the constraints map
        %   This method is a template for populating an entry in the
        %   constraints map. Each entry contains a name, type
        %   (specified as an enumeration), the constraint object, the
        %   stored constraint state details (violations), and the
        %   state. The state is set based on the values of the state
        %   details and whether or not the constraint is enabled.

            entryStruct = struct();
            entryStruct.Name = constraintName;
            entryStruct.Type = constraintType;
            entryStruct.Constraint = constraint;
            entryStruct.StateDetails = constraintStateDetails;

            % Update the constraint state
            if isDisabled
                entryStruct.State = robotics.ikdesigner.internal.model.ConstraintState.Disabled;
            elseif isempty(constraintStateDetails)
                entryStruct.State = robotics.ikdesigner.internal.model.ConstraintState.Unset;
            elseif obj.isConstraintMet(constraintStateDetails)
                entryStruct.State = robotics.ikdesigner.internal.model.ConstraintState.Pass;
            else
                entryStruct.State = robotics.ikdesigner.internal.model.ConstraintState.Fail;
            end
        end

        function isMet = isConstraintMet(constraintStateDetails)
        %isConstraintMet Check if a constraint is met given the ConstraintViolations output
        %   If all the values of constraint violations are sufficiently
        %   small, then the constraint is met. Here, an arbitrary but
        %   constant value is used.

            isMet = all(abs(constraintStateDetails) < robotics.ikdesigner.internal.constants.Data.CONSTRAINTPASSTOLERANCE);
        end

        function [constraintTypes, constraintTypeEnums]  = generateConstraintTypes(constraintTargets)
        %generateConstraintTypes Generate constraint types from a cell array of GIK constraint objects
        %   The GIK solver acknowledges constraints via two properties:
        %   the Constraints property, which contains a cell array of
        %   valid constraint objects, and the ConstraintInputs
        %   property, which is a corresponding cell array of strings
        %   indicating the constraint type. This function maps user-provided
        %   constraints to their associated type strings.

            constraintTypes = cell(1, numel(constraintTargets));
            constraintTypeEnums = cell(1, numel(constraintTargets));
            for i = 1:numel(constraintTypes)
                switch class(constraintTargets{i})
                  case 'constraintAiming'
                    constraintTypes{i} = 'aiming';
                    constraintTypeEnums{i} = robotics.ikdesigner.internal.toolstrip.ConstraintType.Aiming;
                  case 'constraintCartesianBounds'
                    constraintTypes{i} = 'cartesian';
                    constraintTypeEnums{i} = robotics.ikdesigner.internal.toolstrip.ConstraintType.Cartesian;
                  case 'constraintJointBounds'
                    constraintTypes{i} = 'joint';
                    constraintTypeEnums{i} = robotics.ikdesigner.internal.toolstrip.ConstraintType.JointBounds;
                  case 'constraintPoseTarget'
                    constraintTypes{i} = 'pose';
                    constraintTypeEnums{i} = robotics.ikdesigner.internal.toolstrip.ConstraintType.Pose;
                end
            end
        end

        function errorString = validateTreeContainsBody(propName, tree, constraint)
        %validateTreeContainsBody Verify that a body name can be found in a rigidbodytree object

            errorString = string.empty;
            if isprop(constraint, propName)
                constraintBodyName = constraint.(propName);
                try
                    tree.TreeInternal.findBodyIndexByName(constraintBodyName);
                catch ME
                    errorString = string(ME.message);
                end
            end
        end

        function errorString = validatePropertyDimensions(propName, expSize, constraint)
        %validatePropertyDimensions Verify that a property has the expected dimensions

            errorString = string.empty;
            constraintPropVal = constraint.(propName);
            try
                validateattributes(constraintPropVal,{'numeric'}, {'size', expSize}, 'inverseKinematicsDesigner', propName);
            catch ME
                errorString = ME.message;
            end

        end

        function errorString = createConstraintImportErrorString(constraintName, validationErrorStrings)
        %createConstraintImportErrorString Generate a string that concatenates a list of error strings

            errorString = constraintName + ":";
            for i = 1:numel(validationErrorStrings)
                errorString = errorString + newline + "- " + validationErrorStrings(i);
            end
        end

        function [constraintName, constraintType, constraintData] = getConstraintData(obj, constraintKey)
        %getConstraintData Return key constraint data values

            constraintsMapEntry = obj.ConstraintsMap(constraintKey);

            % Create the right type of constraint with the correct data and
            % send it to the controller to be routed to the appropriate
            % view(s)
            constraintName = constraintsMapEntry.Name;
            constraintType = constraintsMapEntry.Type;
            constraintData = obj.populateConstraintDataStruct(constraintKey);

        end
    end
end
