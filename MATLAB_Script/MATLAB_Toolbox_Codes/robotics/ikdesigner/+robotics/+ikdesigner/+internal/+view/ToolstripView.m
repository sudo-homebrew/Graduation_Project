classdef ToolstripView <  robotics.ikdesigner.internal.view.View
%This function is for internal use only. It may be removed in the future.

%ToolstripView creates the main toolstrip view object

%   Copyright 2021 The MathWorks, Inc.

    events
        %CheckAllConfigsCollisions Indicate to controller to have model check all stored configurations for collision
        CheckAllConfigsCollisions

        %CheckCurrentConfigCollisions Indicate to controller to have model check current configuration for collision
        CheckCurrentConfigCollisions

        %IncludeSelfCollisions Tell controller to enable self-collision checking in the model
        IncludeSelfCollisions

        %IgnoreSelfCollisions Tell controller to disable self-collision checking in the model
        IgnoreSelfCollisions

        %CollisionFileAdded Collision data has been imported in the view
        CollisionFileAdded

        %ConfigurationsAdded Configurations have been imported in the view
        ConfigurationsAdded

        %ConstraintEdited Indicate that a constraint has been added/edited
        %   This event indicates that a constraint with a given key (new or
        %   existing) has been edited. The event passes this data to the
        %   controller so it can be added to the correct model.
        ConstraintEdited

        %ConstraintsAdded Constraints have been imported in the view
        ConstraintsAdded

        %ConstraintVisualsChange Indicate that constraint draft is edited
        %   This event is used to communicate changes to the constraint --
        %   both applied and unapplied -- to the controller, which then
        %   routes it to the models/view and updates the associated
        %   constraint overlay visuals.
        ConstraintVisualsChange

        %MarkerBodySelected Indicate that the user has selected a new marker body
        MarkerBodySelected

        %NewSessionRBTSelected Indicate that a new session is requested
        %   This event triggers a new session and provides the rigid body
        %   tree object needed to create the session.
        NewSessionRBTSelected

        %ObjectSelectionChange Indicate to controller that a new object has been selected
        ObjectSelectionChange

        %RequestAddConstraint Request a new constraint event from the model
        %   This method indicates that a user has clicked on the "new
        %   constraint" button, which triggers model actions.
        RequestAddConstraint

        %RequestEditConstraint Request an edit constraint event from the model
        %   This method indicates that a user has clicked on the "edit
        %   constraint" button, which triggers model actions.
        RequestEditConstraint

        %RequestEditMarkerPoseConstraint Request to edit the marker pose constraint
        RequestEditMarkerPoseConstraint

        %RequestUpdatedIKSolution Request that the model update the visible IK solution
        RequestUpdatedIKSolution

        %RequestSaveSession Request that the app model save the session
        RequestSaveSession

        %RequestSaveAsSession Request that the model save session with a new file name
        RequestSaveAsSession

        %RequestLoadSession Request that the model load a session from file
        RequestLoadSession

        %RequestAppBusy Request the model to make the app window busy
        RequestAppBusy

        %RequestAppNotBusy Request the model to make the app window not busy
        RequestAppNotBusy

        %RequestUIAlert Create a UI alert
        RequestUIAlert

        %RequestSolverDataExport Request the model to export solver/constraint data
        RequestSolverDataExport

        %RequestConfigDataExport Request the model to export configurations data
        RequestConfigDataExport

        %SolverAdded Indicate that a solver has been imported
        SolverAdded

        %SolverSettingsEdited Indicate that the user has made and applied changes to the solver settings
        SolverSettingsEdited

        RequestDisableMarker

        RequestEnableMarker

        RequestUpdateMarkerPose

        RequestLastSolutionInfo

        RequestAppSessionChange
    end

    properties (SetAccess = {?robotics.ikdesigner.internal.view.View, ?matlab.unittest.TestCase})

        %Parent Handle to appcontainer
        Parent

        %TabGroup Tab group containing all the tabs
        TabGroup

        %TabTagMap Containers.Map object that relates tab tags to objects
        TabTagMap

        %InverseKinematicsTab Primary tab
        InverseKinematicsTab

        %SolverTab Solver tab
        SolverTab

        %ConstraintTab Handle to the tab currently shown as the constraint
        %   When the toolstrip is in the constraint mode, this property is
        %   a handle to the active tab. When the toolstrip is in the
        %   default mode, this property is empty.
        ConstraintTab

        %DefaultConstraintTab Default (generic) constraint tab, which does not display any settings
        DefaultConstraintTab

        %AimingConstraintTab Aiming constraint tab
        AimingConstraintTab

        %CartesianBoundsConstraintTab Cartesian bounds constraint tab
        CartesianBoundsConstraintTab

        %PoseConstraintTab Pose constraint tab
        PoseConstraintTab

        %JointBoundsConstraintTab Joint bounds constraint tab
        JointBoundsConstraintTab

        %ActiveConstraintName Constraint name displayed in the constraint tab
        %   Since the constraint tab consists of many objects, the name of
        %   the constraint under edit is instead stored here in the
        %   toolstrip, since this object is persistent even while the
        %   displayed tabs change.
        %   TODO: (g2570347) Move this somewhere that isn't just the toolstrip view.
        ActiveConstraintName

        %ActiveConstraintKey Key associated with the constraint being edited in the constraint tab
        %   Since the constraint tab consists of many objects, the key of
        %   the constraint under edit is instead stored here in the
        %   toolstrip, since this object is persistent even while the
        %   displayed tabs change.
        %   TODO: (g2570347) Move this somewhere that isn't just the toolstrip view.
        ActiveConstraintKey

    end

    properties (Access = {?matlab.unittest.TestCase, ?robotics_tests.app.ikdesigner.ToolstripTester})
        FileSelector

        DataImportView

        DataExportView
    end

    properties (Constant)
        DefaultSolverObj = inverseKinematics;
    end

    properties (Dependent)
        %ToolstripMode Returns an enum indicating the current toolstrip mode
        ToolstripMode
    end

    properties (Dependent, SetAccess = {?robotics.ikdesigner.internal.controller.AppStateController, ?matlab.unittest.TestCase})
        AllowSessionChange
    end

    properties (SetAccess = ?robotics.ikdesigner.internal.toolstrip.Tab)
        CanProceed
    end

    methods
        function obj = ToolstripView(container)
        %TOOLSTRIP Object constructor

            arguments
                container matlab.ui.container.internal.AppContainer
            end

            % Set the parent and initialize a context
            obj.Parent = container;

            % Assign views for loading files and data
            obj.FileSelector = robotics.ikdesigner.internal.view.FileSelector(container);
            obj.DataImportView = robotics.ikdesigner.internal.view.DataImportView(container, obj);
            obj.DataExportView = robotics.ikdesigner.internal.view.DataExportView(container, obj);

            %Create the main tab group
            obj.buildGlobalTabGroup;

            % Initialize allowed session change state
            obj.CanProceed = false;
        end

        function setup(obj, robot, ikSolver, rigidBodyKeyMap)
        %setup Set up the toolstrip tabs with new session data

        % Initialize the tabs using parameters from the robot & solver
            obj.InverseKinematicsTab.setup(robot, rigidBodyKeyMap);
            obj.SolverTab.setup(ikSolver);

            % Set up properties that are solver dependent
            obj.AimingConstraintTab.setup(robot, rigidBodyKeyMap);
            obj.CartesianBoundsConstraintTab.setup(robot, rigidBodyKeyMap);
            obj.JointBoundsConstraintTab.setup(robot);
            obj.PoseConstraintTab.setup(robot, rigidBodyKeyMap);

            % Initially the main tabs should be visible
            obj.updateToolstripMode(robotics.ikdesigner.internal.toolstrip.ModeEnum.DefaultMode);

        end

        function initialize(obj)
        %initialize toolstrip button status

            % Initialize individual tabs
            obj.InverseKinematicsTab.initialize();
            obj.SolverTab.initialize();
            obj.DefaultConstraintTab.initialize();
            obj.AimingConstraintTab.initialize();
            obj.CartesianBoundsConstraintTab.initialize();
            obj.JointBoundsConstraintTab.initialize();
            obj.PoseConstraintTab.initialize();

            % Initialize the data import view
            obj.DataImportView.initialize();
            obj.DataExportView.initialize();
        end

        function sendNotification(obj, eventName, eventData)
        %sendNotification Generic notification wrapper

            if isempty(eventData)
                notify(obj, eventName);
            else
                notify(obj, eventName, eventData);
            end

        end

        function closeOpenFigures(obj)
            %closeOpenFigures Close open figures that are not already handled by app container infrastructure
            %   When the app is closed, any external (non-appcontainer)
            %   figures that were spawned by the toolstrip views must also
            %   be closed.

            % Close figures created by the IK Tab
            obj.InverseKinematicsTab.closeOpenNonModalFigures();
        end

        function initiateSaveAsSession(obj)
        %initiateSaveAsSession Initiate the save-as dialog
        %   This method is called by the client to initiate the save as
        %   dialog and save the current app session.

            obj.InverseKinematicsTab.initiateSaveAsSession();

        end

        function setSessionSavedTo(obj, isSaved)
            %setSessionSavedTo Define view behaviors depending on whether the session is saved or not
            obj.InverseKinematicsTab.applySavedSessionIcons(isSaved);
        end

        function editNewConstraint(obj, constraintKey, constraintName)
        %editNewConstraint Edit a new constraint with the provided key and default name

        % With a new constraint, all tabs should take their default
        % values
            obj.resetAllConstraintTabsToDefaultState;

            % Start with the default tab
            tabType = robotics.ikdesigner.internal.constants.Toolstrip.DefaultConstraintTabTag;

            % Update the constraint's key and name
            obj.ActiveConstraintKey = constraintKey;
            obj.ActiveConstraintName = constraintName;

            % Update the toolstrip display
            obj.updateToolstripViewForConstraintEdit(tabType);
        end

        function editExistingConstraint(obj, constraintKey, constraintName, constraintType, constraintData)
        %editExistingConstraint Update the toolstrip to edit an existing constraint
        %   Edit a constraint with the provided key, name, type, and
        %   constraint data.

        % TODO: (g2570347) Consider moving this to a standalone interface

        % Populate the fields so that the tab has the correct data
            obj.populateConstraintDataFields(constraintType, constraintData);
            tabType = obj.mapConstraintTypeToTabType(constraintType);

            % Update the constraint's key and name
            obj.ActiveConstraintKey = constraintKey;
            obj.ActiveConstraintName = constraintName;

            % Update the toolstrip display
            obj.updateToolstripViewForConstraintEdit(tabType);
        end


        function updateConstraintButtonEnabledState(obj, isEnabled)
        %updateConstraintButtonEnabledState Enable or disable the "Edit Constraint" button

        % TODO: (g2570347) Consider moving this to a standalone interface

            obj.InverseKinematicsTab.EditConstraintButton.Enabled = isEnabled;
        end

        function updateSolverReportFromInfo(obj,info)
            obj.InverseKinematicsTab.SolverInfoView.updateFromSolutionInfo(info);
        end

        function updateToolstripMode(obj, toolstripModeEnum)
        %updateToolstripMode Update the toolstrip mode
        %   Set the mode of the toolstrip, indicated by an enum. There
        %   are two modes: the default mode that the user sees when the
        %   tab is loaded, and the constraint mode.

            switch toolstripModeEnum
              case robotics.ikdesigner.internal.toolstrip.ModeEnum.DefaultMode

                % Update the tab state
                if ~isempty(obj.ConstraintTab)
                    obj.ConstraintTab.hide();
                    obj.ConstraintTab = [];
                end
                obj.InverseKinematicsTab.show();
                obj.SolverTab.show();
                obj.InverseKinematicsTab.select();

              case robotics.ikdesigner.internal.toolstrip.ModeEnum.ConstraintMode

                % Update the tab state
                obj.ConstraintTab = obj.DefaultConstraintTab;
                obj.ConstraintTab.show();
                obj.InverseKinematicsTab.hide();
                obj.SolverTab.hide();
            end
        end

        function updateToolstripTabDisplay(obj, tabTagsToHide, tabTagsToShow, tabTagToSelect, isVisibleTabAConstraint)
        %updateToolstripTabDisplay Update the displayed tabs in the toolstrip
        %   This method takes three inputs: the cell array of tab tags
        %   to be hidden, the cell array of tab tags to be shown, and
        %   if specified, the tab tag (a string) to be selected. The
        %   last value is a boolean indicating whether the resultant
        %   visible tab is a constraint tab or not.

        % Iterate over the array of tabs to hide and remove them from
        % the toolstrip
            for i = 1:numel(tabTagsToHide)
                tabToHide = obj.TabTagMap(tabTagsToHide{i});
                tabToHide.hide();
            end

            % Iterate over the array of tabs to show and add them to
            % the toolstrip
            for i = 1:numel(tabTagsToShow)
                tabToShow = obj.TabTagMap(tabTagsToShow{i});
                tabToShow.show();
            end

            % If specified, bring the selected tab to the forefront
            if ~isempty(tabTagToSelect)
                tabToSelect = obj.TabTagMap(tabTagToSelect);
                tabToSelect.select();
            end

            % Update the constraint tab property to reflect the current
            % display
            if isVisibleTabAConstraint
                obj.ConstraintTab = obj.TabTagMap(obj.TabGroup.SelectedTab.Tag);
            else
                obj.ConstraintTab = [];
            end
        end

        function updateMarkerBodySelection(obj, bodyKey)
        %updateMarkerBodySelection Update the selected marker body in the toolstrip

            obj.InverseKinematicsTab.updateMarkerBodyDropdown(bodyKey);

        end

        function updateSolverTabValues(obj, solver)
        %updateSolverTabValues Update the solver tab values given new solver parameters

            obj.SolverTab.populateSolverTab(solver.SolverAlgorithm, solver.SolverParameters);

        end

        function exportSolverData(obj, solver, constraintValues, constraintNames)
        %exportSolverData Create a UI to export solver and constraints
        %   This method is called by the client controller which
        %   provides the values to be exported. The toolstrip then
        %   facilitates GUI creation so that the user may ultimately
        %   choose which variables they want to export.

            obj.InverseKinematicsTab.exportSolverData(solver, constraintValues, constraintNames);
        end

        function exportConfigData(obj, configValues, configNames)
        %exportConfigData Create a UI to export configurations
        %   This method is called by the client controller which
        %   provides the values to be exported. The toolstrip then
        %   facilitates GUI creation so that the user may ultimately
        %   choose which variables they want to export.

            obj.InverseKinematicsTab.exportConfigData(configValues, configNames);
        end

        function tsMode = get.ToolstripMode(obj)
        %get.ToolstripMode Return an enumeration indicating the toolstrip mode
        %   This method just checks if the IK or solver tab are
        %   displayed to see whether the default toolstrip mode is
        %   active. The toolstrip mode more precisely corresponds to a
        %   greater set of rules, but this is sufficient to determine
        %   mode in part because the selected tab is usually the last
        %   item that is set.

            if (obj.TabGroup.SelectedTab == obj.InverseKinematicsTab.TabHandle) ...
                    || (obj.TabGroup.SelectedTab == obj.SolverTab.TabHandle)
                tsMode = robotics.ikdesigner.internal.toolstrip.ModeEnum.DefaultMode;
            else
                tsMode = robotics.ikdesigner.internal.toolstrip.ModeEnum.ConstraintMode;
            end
        end

        function set.AllowSessionChange(obj, isAllowable)
            obj.InverseKinematicsTab.AllowSessionChange = isAllowable;
            obj.CanProceed = true;
        end
    end

    methods (Access = ?matlab.unittest.TestCase)
        function updateToolstripViewForConstraintEdit(obj, tabType)
        %updateToolstripViewForConstraintEdit Move to constraint-edit mode with the tab of the specified type
        %   This method updates the toolstrip so that it displays the
        %   selected constraint tab type in the constraints mode. This
        %   method assumes that the tab constraints have already been
        %   populated with the correct data if needed.

        % Update the user-facing tab views. Start with the mode.
            obj.updateToolstripMode(1);

            % Make sure the correct tab is shown
            obj.ConstraintTab.assignSpecificConstraintTab(tabType);
        end
    end

    methods (Access = private)
        function buildGlobalTabGroup(obj)
        %buildGlobalTabGroup Create a tab group and add the tabs to it

            obj.TabGroup = matlab.ui.internal.toolstrip.TabGroup();
            obj.TabGroup.Tag = "Toolstrip";
            obj.TabGroup.Contextual = false;
            obj.Parent.addTabGroup(obj.TabGroup);

            % Create the main tabs
            obj.InverseKinematicsTab = robotics.ikdesigner.internal.toolstrip.IKTab(obj.FileSelector, obj.DataImportView, obj.DataExportView, obj);
            obj.SolverTab = robotics.ikdesigner.internal.toolstrip.SolverTab(obj);

            % Create the constraint tabs
            defaultTabTag = robotics.ikdesigner.internal.constants.Toolstrip.DefaultConstraintTabTag;
            obj.DefaultConstraintTab = robotics.ikdesigner.internal.toolstrip.ConstraintTab(obj, defaultTabTag);
            obj.AimingConstraintTab = robotics.ikdesigner.internal.toolstrip.AimingConstraintTab(obj);
            obj.CartesianBoundsConstraintTab = robotics.ikdesigner.internal.toolstrip.CartesianBoundsConstraintTab(obj);
            obj.JointBoundsConstraintTab = robotics.ikdesigner.internal.toolstrip.JointBoundsConstraintTab(obj);
            obj.PoseConstraintTab = robotics.ikdesigner.internal.toolstrip.PoseConstraintTab(obj);

            % Associate tabs with their tags
            obj.createTabTagMap();

            % Initially the main tabs should be visible
            obj.updateToolstripMode(robotics.ikdesigner.internal.toolstrip.ModeEnum.DefaultMode);

        end

        function createTabTagMap(obj)
        %createTabTagMap Create a map that relates tab tags to objects
        %   This object provides a way to get a handle to the tab
        %   object given its tag. While the toolstrip can already be
        %   queried for tags to get to the objects, this returns the
        %   robotics tab object, not the underlying uitab.
        %   Additionally, it is not always possible to search the
        %   toolstrip or tab group for the tabs because the uitab is
        %   only actually a child of the toolstrip when it is visible;
        %   all other times it is an unconnected object.

            obj.TabTagMap = containers.Map.empty;

            tabMapKeys = [...
                robotics.ikdesigner.internal.constants.Toolstrip.IKTabTag; ...
                robotics.ikdesigner.internal.constants.Toolstrip.SolverTabTag; ...
                robotics.ikdesigner.internal.constants.Toolstrip.DefaultConstraintTabTag; ...
                robotics.ikdesigner.internal.constants.Toolstrip.AimingConstraintTabTag; ...
                robotics.ikdesigner.internal.constants.Toolstrip.CartesianBoundsConstraintTabTag; ...
                robotics.ikdesigner.internal.constants.Toolstrip.JointBoundsConstraintTabTag; ...
                robotics.ikdesigner.internal.constants.Toolstrip.PoseTargetConstraintTabTag ...
                         ];

            tabMapValues = {...
                obj.InverseKinematicsTab; ...
                obj.SolverTab; ...
                obj.DefaultConstraintTab; ...
                obj.AimingConstraintTab; ...
                obj.CartesianBoundsConstraintTab; ...
                obj.JointBoundsConstraintTab; ...
                obj.PoseConstraintTab ...
                           };
            obj.TabTagMap = containers.Map(tabMapKeys, tabMapValues);
        end

        function populateConstraintDataFields(obj, constraintType, constraintData)
        %populateConstraintDataFields Populate the user-facing constraint tab fields given existing event data from the model

        % Populate constraint-specific data
            switch constraintType
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Aiming
                obj.AimingConstraintTab.populateTabFieldsFromDataStruct(constraintData);
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Cartesian
                obj.CartesianBoundsConstraintTab.populateTabFieldsFromDataStruct(constraintData);
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.JointBounds
                obj.JointBoundsConstraintTab.populateTabFieldsFromDataStruct(constraintData);
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Pose
                obj.PoseConstraintTab.populateTabFieldsFromDataStruct(constraintData);
            end

        end

        function resetAllConstraintTabsToDefaultState(obj)
        %resetAllConstraintTabsToDefaultState Reset the constraint tabs to their default values

            obj.AimingConstraintTab.resetToDefaultState;
            obj.CartesianBoundsConstraintTab.resetToDefaultState;
            obj.JointBoundsConstraintTab.resetToDefaultState;
            obj.PoseConstraintTab.resetToDefaultState;
        end
    end

    methods (Static, Access = private)
        function tabType = mapConstraintTypeToTabType(constraintType)
        %mapConstraintTypeToTabType Get the constraint tab type from the constraint type

            switch constraintType
              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Aiming
                tabType = robotics.ikdesigner.internal.constants.Toolstrip.AimingConstraintTabTag;

              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Cartesian
                tabType = robotics.ikdesigner.internal.constants.Toolstrip.CartesianBoundsConstraintTabTag;

              case robotics.ikdesigner.internal.toolstrip.ConstraintType.JointBounds
                tabType = robotics.ikdesigner.internal.constants.Toolstrip.JointBoundsConstraintTabTag;

              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Pose
                tabType = robotics.ikdesigner.internal.constants.Toolstrip.PoseTargetConstraintTabTag;

              case robotics.ikdesigner.internal.toolstrip.ConstraintType.Default
                tabType = robotics.ikdesigner.internal.constants.Toolstrip.DefaultConstraintTabTag;
            end

        end
    end
end
