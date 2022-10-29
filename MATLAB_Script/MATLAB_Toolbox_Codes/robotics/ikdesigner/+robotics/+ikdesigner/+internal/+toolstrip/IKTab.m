classdef IKTab < robotics.ikdesigner.internal.toolstrip.Tab
%This class is for internal use only. It may be removed in the future.

%IKTab Tab view that for the main tab in the inverse kinematics designer toolstrip

%   Copyright 2021-2022 The MathWorks, Inc.

    properties (SetAccess = {?robotics.ikdesigner.internal.view.View, ?matlab.unittest.TestCase})
        % The first set of properties contains handles to objects
        % associated with the general toolstrip

        %DataImportView Handle to the view object that provides a GUI for loading data from the workspace into the app
        DataImportView

        %DataExportView Handle to the view object that provides a GUI for exporting data to the workspace
        DataExportView

        %FileSelectorView Handle to the view object that provides a GUI for loading data from file into the app
        FileSelectorView

        SolverInfoView
    end

    properties (SetAccess = {?robotics.ikdesigner.internal.view.View, ?matlab.unittest.TestCase})
        % These properties contain objects in the toolstrip. Note that it
        % isn't necessary to store handles for all components; instead, it
        % is only necessary to have handles to objects when the object can
        % also be modified after construction by the toolstrip or one of
        % its clients. Additionally, the sections are stored for
        % convenience.

        %FileSection File
        FileSection

        %SceneSection Scene section
        SceneSection

        %IKSection Inverse Kinematics section
        IKSection

        %MarkerConstraintSection Marker Constraint section
        MarkerConstraintSection

        %ConfigurationsSection Configurations section
        ConfigurationsSection

        %CollisionsSection Collisions section
        CollisionsSection

        % Export section
        ExportSection

        %EditConstraintButton Button to edit constraints
        %   The edit constraint button can be enabled or disabled
        %   independently of the rest of the toolstrip based on the state
        %   of the constraint selection.
        EditConstraintButton

        %NewSessionButton New Session buttonADD_24
        %   The new session button can be enabled independently of the
        %   reset of the toolstrip.
        NewSessionButton

        %OpenSessionButton Open Session button
        %   The open session button can be enabled independently of the
        %   reset of the toolstrip.
        OpenSessionButton

        %SaveButton Save and Save As buttons
        SaveButton

        %MarkerBodyDropdown Handle to the dropdown containing the list of available marker bodies
        %   The marker body dropdown is initialized with all the bodies in
        %   the robot at each session start. It can also be updated by a
        %   view when the marker body is changed.
        MarkerBodyDropdown

        %IgnoreSelfCollisionsCheckbox Checkbox that toggles self-collisions
        IgnoreSelfCollisionsCheckbox
    end

    properties
        AllowSessionChange
    end

    properties (Constant)
        ADDCOLLISIONICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'addCollisionObject.png')
        CHECKCOLLISIONICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'checkCollisions.png')
        ADDCONSTRAINTICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'addConstraint.png')
        EDITCONSTRAINTICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'editConstraint.png')
        POSECONSTRAINTICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'pose.png')
        SOLVERREPORTICON = fullfile(robotics.ikdesigner.internal.constants.Data.CUSTOMICONSDIRECTORY, 'report_16.png')
    end

    properties (Constant, Access = private)
        %New session window title and instruction text
        NEWSESSIONWINDOWTITLE = string(message('robotics:ikdesigner:dataimportexport:NewSessionWindowName'))
        NEWSESSIONINSTRUCTIONTEXT = string(message('robotics:ikdesigner:dataimportexport:NewSessionInstructionText'))
        RBTLABEL = string(message('robotics:ikdesigner:dataimportexport:RigidBodyTreeLabel'))
        NEWSESSIONVARSTABLELABEL = string(message('robotics:ikdesigner:dataimportexport:NewSessionAvailableVarsLabel'))
        NEWSESSIONOKBUTTONLABEL = string(message('robotics:ikdesigner:dataimportexport:NewSessionOKButtonLabel'))
        NEWSESSIONDOCID = "ikDesignerNewSession"

        %Add collision window title and instruction text
        ADDCOLLISIONWINDOWTITLE = string(message('robotics:ikdesigner:dataimportexport:AddCollisionWindowName'))
        ADDCOLLISIONINSTRUCTIONTEXT = string(message('robotics:ikdesigner:dataimportexport:AddCollisionInstructionText'))
        COLLISIONVARSTABLELABEL = string(message('robotics:ikdesigner:dataimportexport:CollisionAvailableVarsLabel'))
        ADDBUTTONLABEL = string(message('robotics:ikdesigner:dataimportexport:AddButtonLabel'))
        ADDCOLLISIONDOCID = "ikDesignerAddCollision"

        %Import solver window title and instruction text
        IMPORTSOLVERWINDOWTITLE = string(message('robotics:ikdesigner:dataimportexport:ImportSolverWindowName'))
        IMPORTSOLVERINSTRUCTIONTEXT = string(message('robotics:ikdesigner:dataimportexport:ImportSolverInstructionText'))
        SOLVERVARSTABLELABEL = string(message('robotics:ikdesigner:dataimportexport:IKSolversAvailableVarsLabel'))
        UIIMPORTBUTTONLABEL = string(message('robotics:ikdesigner:dataimportexport:UIImportButtonLabel'))
        IMPORTSOLVERDOCID = "ikDesignerImportSolver"

        %Import configurations window title and instruction text
        IMPORTCONFIGSWINDOWTITLE = string(message('robotics:ikdesigner:dataimportexport:ImportConfigurationsWindowName'))
        IMPORTCONFIGSINSTRUCTIONTEXT = string(message('robotics:ikdesigner:dataimportexport:ImportConfigurationsInstructionText'))
        CONFIGSVARSTABLELABEL = string(message('robotics:ikdesigner:dataimportexport:ConfigsAvailableVarsLabel'))
        IMPORTCONFIGSDOCID = "ikDesignerImportConfigs"

        %Import constraints window title and instruction text
        IMPORTCONSTRAINTSWINDOWTITLE = string(message('robotics:ikdesigner:dataimportexport:ImportConstraintsWindowName'))
        IMPORTCONSTRAINTSINSTRUCTIONTEXT = string(message('robotics:ikdesigner:dataimportexport:ImportConstraintsInstructionText'))
        CONSTRAINTSVARSTABLELABEL = string(message('robotics:ikdesigner:dataimportexport:ConstraintsAvailableVarsLabel'))
        IMPORTCONSTRAINTSDOCID = "ikDesignerImportConstraints"

        %Export configurations window title and instruction text
        EXPORTCONFIGSWINDOWTITLE = string(message('robotics:ikdesigner:dataimportexport:ExportConfigsWindowName'))
        EXPORTCONFIGSINSTRUCTIONTEXT = string(message('robotics:ikdesigner:dataimportexport:ExportConfigsInstructionText'))
        EXPORTCONFIGSDOCID = "ikDesignerExportConfigs"

        %Export solver window title and instruction text
        EXPORTSOLVERWINDOWTITLE = string(message('robotics:ikdesigner:dataimportexport:ExportSolverWindowName'))
        EXPORTSOLVERINSTRUCTIONTEXT = string(message('robotics:ikdesigner:dataimportexport:ExportSolverInstructionText'))
        UIEXPORTBUTTONLABEL= string(message('robotics:ikdesigner:dataimportexport:UIExportButtonLabel'))
        EXPORTSOLVERDOCID = "ikDesignerExportSolver"

        %File section labels
        FILESECTIONLABEL = string(message('robotics:ikdesigner:toolstrip:FileSectionLabel'))
        NEWSESSIONBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:NewSessionButtonLabel'))
        OPENSESSIONBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:OpenSessionButtonLabel'))
        SAVESESSIONBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:SaveSessionButtonLabel'))
        SAVEBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:SaveButtonLabel'))
        SAVEASBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:SaveAsButtonLabel'))
        IMPORTBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:ImportButtonLabel'))
        IMPORTSOLVERBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:ImportSolverButtonLabel'))
        IMPORTCONSTRAINTSBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:ImportConstraintsButtonLabel'))
        IMPORTCONFIGSBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:ImportConfigurationsButtonLabel'))

        %File section tooltips
        NEWSESSIONBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:NewSessionButtonTooltip'))
        OPENSESSIONBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:OpenSessionButtonTooltip'))
        SAVESESSIONBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:SaveSessionButtonTooltip'))
        IMPORTBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:ImportButtonTooltip'))
        IMPORTSOLVERBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:ImportSolverButtonTooltip'))
        IMPORTCONSTRAINTSBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:ImportConstraintsButtonTooltip'))
        IMPORTCONFIGSBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:ImportConfigurationsButtonTooltip'))

        %Scene section labels
        SCENESECTIONLABEL = string(message('robotics:ikdesigner:toolstrip:SceneSectionLabel'))
        ADDCOLLISIONBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:AddCollisionButtonLabel'))

        %Scene section tooltips
        ADDCOLLISIONBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:AddCollisionButtonTooltip'))

        %Inverse Kinematics section labels
        IKSECTIONLABEL = string(message('robotics:ikdesigner:toolstrip:IKSectionLabel'))
        ADDCONSTRAINTBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:AddConstraintButtonLabel'))
        EDITCONSTRAINTBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:EditConstraintButtonLabel'))
        REFRESHSOLVERBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:RefreshSolverButtonLabel'))
        SOLVERSETTINGSBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:SolverSettingsButtonLabel'))
        REPORTSTATUSBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:ReportStatusButtonLabel'))

        %Inverse Kinematics section tooltips
        ADDCONSTRAINTBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:AddConstraintButtonTooltip'))
        EDITCONSTRAINTBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:EditConstraintButtonTooltip'))
        REFRESHSOLVERBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:RefreshSolverButtonTooltip'))
        SOLVERSETTINGSBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:SolverSettingsButtonTooltip'))
        REPORTSTATUSBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:ReportStatusButtonTooltip'))

        %Marker section labels
        MARKERSECTIONLABEL = string(message('robotics:ikdesigner:toolstrip:MarkerConstraintSectionLabel'))
        MARKERBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:MarkerButtonLabel'))
        MARKERBODYLABEL = string(message('robotics:ikdesigner:toolstrip:MarkerBodyDropdownLabel'))

        %Marker section tooltips
        MARKERBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:MarkerButtonTooltip'))
        MARKERBODYTOOLTIP = string(message('robotics:ikdesigner:toolstrip:MarkerBodyDropdownTooltip'))

        %Collisions section labels
        COLLISIONSSECTIONLABEL = string(message('robotics:ikdesigner:toolstrip:CollisionsSectionLabel'))
        CHECKCOLLISIONSBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:CheckCollisionsButtonLabel'))
        CHECKCURRCOLLISIONSBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:CheckCurrConfigCollisionButtonLabel'))
        CHECKALLCOLLISIONSBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:CheckAllConfigsCollisionButtonLabel'))
        COLLISIONOPTSLABEL = string(message('robotics:ikdesigner:toolstrip:CollisionOptionsSectionLabel'))
        IGNORESELFCOLLISIONCHECKBOXLABEL = string(message('robotics:ikdesigner:toolstrip:IgnoreSelfCollisionCheckboxLabel'))

        %Collisions section tooltips
        CHECKCOLLISIONSBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:CheckCollisionsButtonTooltip'))
        CHECKCURRCOLLISIONSBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:CheckCurrConfigCollisionButtonTooltip'))
        CHECKALLCOLLISIONSBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:CheckAllConfigsCollisionButtonTooltip'))
        IGNORESELFCOLLISIONCHECKBOXTOOLTIP = string(message('robotics:ikdesigner:toolstrip:IgnoreSelfCollisionCheckboxTooltip'))

        %Export section labels
        EXPORTSECTIONLABEL = string(message('robotics:ikdesigner:toolstrip:ExportSectionLabel'))
        EXPORTBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:ExportButtonLabel'))
        EXPORTSOLVERBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:ExportSolverButtonLabel'))
        EXPORTCONFIGSBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:ExportConfigsButtonLabel'))

        %Export section tooltips
        EXPORTBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:ExportButtonTooltip'))
        EXPORTSOLVERBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:ExportSolverButtonTooltip'))
        EXPORTCONFIGSBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:ExportConfigsButtonTooltip'))
    end

    methods
        function obj = IKTab(fileSelectorView, dataImportView, dataExportView, toolstrip)
        %IKTab Constructor

        % Construct the tab and associate it with the toolstrip
            obj@robotics.ikdesigner.internal.toolstrip.Tab(toolstrip, "Inverse Kinematics", "InverseKinematics");

            % Add sections to the tab
            obj.addFileSection();
            obj.addSceneSection();
            obj.addIKSection();
            obj.addMarkerConstraintSection();
            obj.addCollisionsSection();
            obj.addExportSection();

            % Store the handle to the data import view, which will be used
            % to load data
            obj.DataImportView = dataImportView;
            obj.DataExportView = dataExportView;
            obj.FileSelectorView = fileSelectorView;
            obj.SolverInfoView = robotics.ikdesigner.internal.view.SolverInfoView(obj);

            % Move the tab to its initial state after starting the app
            obj.disableAll;
            obj.NewSessionButton.Enabled = true;
            obj.OpenSessionButton.Enabled = true;
        end

        function setup(obj, robot, rigidBodyKeyMap)
        %setup Set up the tab with new session data

            % Configure the body name drop downs with new values
            obj.initializeBodyNameDropdown(obj.MarkerBodyDropdown, robot, rigidBodyKeyMap, rigidBodyKeyMap.Count);

            % Clear the solver info view window if it is open
            obj.SolverInfoView.hide();

            % Reset the self collision check
            obj.IgnoreSelfCollisionsCheckbox.Value = false;

        end

        function initialize(obj)
        %initialize Initialize the tab for use with current data members

        % Enable all the buttons in the tab
            obj.enableAll;

            % By default no constraint is selected
            obj.EditConstraintButton.Enabled = false;
        end
    end

    % External methods that update the tab may only be accessed through the
    % main toolstrip view or its children, all of which must be view
    % objects.
    methods (Access = ?robotics.ikdesigner.internal.view.View)

        function updateMarkerBodyDropdown(obj, bodyKey)
        %updateMarkerBodyDropdown Update the marker body dropdown to select the marker associated with a given key

            obj.MarkerBodyDropdown.Value = bodyKey;
        end

        function closeOpenNonModalFigures(obj)
            %closeOpenNonModalFigures Close open non-modal figures
            %   When the app is closed, any figures that were spawned by
            %   the IK tab must also be closed.
            
            obj.SolverInfoView.hide();
        end

        function applySavedSessionIcons(obj, isSavedSession)
        %applySavedSessionIcons Change icons when session is saved or not

            if isSavedSession
                obj.SaveButton.Icon = matlab.ui.internal.toolstrip.Icon.SAVE_24;
            else
                obj.SaveButton.Icon = matlab.ui.internal.toolstrip.Icon.SAVE_DIRTY_24;
            end
        end

        function initiateSaveAsSession(obj)
        %initiateSaveAsSession Initiate Save-As workflow
        %   This method brings up the file selector, which prompts the
        %   user to pick a location to save the current session. Once
        %   complete, the selected location is forwarded to the
        %   controller, which completes the process. This method may be
        %   instantiated directly by the tab, or by the toolstrip view.

            % Make sure the app is blocked off
            obj.sendNotification("RequestAppBusy",[]);

            % Open the UI. If the user cancels, the selector will return
            % zero for both outputs. That will end this workflow.
            [isFileReturned, filepath] = obj.FileSelectorView.chooseFileToSave("iksessiondata.mat");
            if isFileReturned
                saveSessionEventData = robotics.ikdesigner.internal.event.FilePathEvent(filepath);
                obj.sendNotification("RequestSaveAsSession", saveSessionEventData);
            else
                obj.sendNotification("RequestAppNotBusy",[]);
            end
        end

        function exportConfigData(obj, configData, configNames)
        %loadConfigurations Load the dialog to import existing configurations

            obj.initializeWindowForModalUI;

            obj.DataExportView.DescriptionText = obj.EXPORTCONFIGSINSTRUCTIONTEXT;
            obj.DataExportView.WindowName = obj.EXPORTCONFIGSWINDOWTITLE;
            obj.DataExportView.OKButtonText = obj.EXPORTBUTTONLABEL;
            obj.DataExportView.HelpDocID = obj.EXPORTCONFIGSDOCID;

            % Open the data export dialog
            obj.DataExportView.createConfigExportView(configData, configNames);
        end

        function exportSolverData(obj, solver, constraintData, constraintNames)
        %loadConfigurations Load the dialog to import existing configurations

            obj.initializeWindowForModalUI;

            obj.DataExportView.DescriptionText = obj.EXPORTSOLVERINSTRUCTIONTEXT;
            obj.DataExportView.WindowName = obj.EXPORTSOLVERWINDOWTITLE;
            obj.DataExportView.OKButtonText = obj.EXPORTBUTTONLABEL;
            obj.DataExportView.HelpDocID = obj.EXPORTSOLVERDOCID;

            % Open the data export dialog
            obj.DataExportView.createSolverConstraintsExportView(solver, constraintData, constraintNames);
        end
    end

    methods (Access = private)

        function notifyCheckCollision(obj, type)
        %notifyCheckCollision Notify the controller of the command to check collisions

            if type == 1
                sendNotification(obj, 'CheckCurrentConfigCollisions', []);
            else
                sendNotification(obj, 'CheckAllConfigsCollisions', []);
            end
        end

        function enableIgnoreSelfCollisions(obj, src, ~)
            %enableIgnoreSelfCollisions Ignore or include self collision checking on the model
            
            if src.Selected
                sendNotification(obj, 'IgnoreSelfCollisions', []);
            else
                sendNotification(obj, 'IncludeSelfCollisions', []);
            end
        end

        function notifyMarkerBodySelection(obj, evt)
        %notifyMarkerBodySelection Notify a change in the body to marker is attached to

            objKey = evt.EventData.NewValue;
            event = robotics.ikdesigner.internal.event.ViewSelectionEventData(objKey, objKey);
            sendNotification(obj, "MarkerBodySelected", event);
        end

        function notifySolverRefresh(obj)
        %notifySolverRefresh Notify the solver that the refresh button has been clicked
        %   This sends a notification to re-run the IK solver and
        %   produce a new result.

            obj.sendNotification('RequestUpdatedIKSolution', []);
        end

        function initiateSaveSession(obj)
        %initiateSaveAsSession Initiate Save workflow
        %   This method forwards the request to re-save the model to
        %   the controller, which completes the process.

        % Ensure the app window is blocked while save is ongoing
            obj.sendNotification("RequestAppBusy",[]);

            % Saving the session requires checking the current saved
            % session state, which is handled outside the toolstrip view
            obj.sendNotification("RequestSaveSession", []);

        end

        function initiateLoadSession(obj)
        %initiateSaveAsSession Initiate load workflow
        %   This method opens the UI so that the user may select a MAT
        %   file to load, then forwards the request to re-save the
        %   model to the controller, which completes the process.

        % Ensure the app window is blocked while load is ongoing

            obj.sendNotification("RequestAppSessionChange", []);
            obj.sendNotification("RequestAppBusy",[]);

            % The UI returns zero for both outputs if the user cancels. In
            % that case, the workflow ends here.
            [isFileReturned, filepath] = obj.FileSelectorView.chooseFileToLoad('.mat');
            if isFileReturned
                loadSessionEvent = robotics.ikdesigner.internal.event.FilePathEvent(filepath);
                obj.sendNotification("RequestLoadSession", loadSessionEvent);
            else
                obj.sendNotification("RequestAppNotBusy",[]);
            end

        end

        function loadRigidBodyTree(obj)
        %loadRigidBodyTree Load the dialog to import a rigid body tree object
        %   This method customizes the data import view and then
        %   creates the UI, which produces a popup window that displays
        %   a filtered view of the workspace. The window filters the
        %   workspace by rigid body tree data type, as set in the
        %   DataTypeFilter of the data import view.

        % Ensure the app window is blocked while load is ongoing

            obj.Parent.CanProceed = false;
            obj.sendNotification("RequestAppSessionChange", []);
            waitfor(obj.Parent, 'CanProceed', true);
            if ~obj.AllowSessionChange
                return;
            end
            obj.initializeWindowForModalUI;

            % Update the DataImportView settings for a new session
            obj.DataImportView.DescriptionText = obj.NEWSESSIONINSTRUCTIONTEXT + newline + newline + obj.RBTLABEL;
            obj.DataImportView.DataTypeFilter = {'rigidBodyTree'};
            obj.DataImportView.DataImportEventName = "NewSessionRBTSelected";
            obj.DataImportView.WindowName = obj.NEWSESSIONWINDOWTITLE;
            obj.DataImportView.TablePanelText = obj.NEWSESSIONVARSTABLELABEL;
            obj.DataImportView.OKButtonText = obj.NEWSESSIONOKBUTTONLABEL;
            obj.DataImportView.HelpDocID = obj.NEWSESSIONDOCID;

            % Open the DataImportView dialog
            [dropdownListText, dropDownListValues] = obj.getRobotList();
            obj.DataImportView.createUIFromList(dropdownListText, dropDownListValues);

            % Update the data validation function
            obj.DataImportView.DataValidationFcn = @(data)obj.validateImportedTreeData(data);
        end

        function loadCollisionObject(obj)
        %loadCollisionObject Load the dialog to import a collision object
        %   This method customizes the data import view and then
        %   creates the UI, which produces a popup window that displays
        %   a filtered view of the workspace. The window filters the
        %   workspace by collision geometry data type, as set in the
        %   DataTypeFilter of the data import view.

            obj.initializeWindowForModalUI;

            % Update the DataImportView settings for a new session
            obj.DataImportView.DescriptionText = obj.ADDCOLLISIONINSTRUCTIONTEXT;
            obj.DataImportView.DataTypeFilter = {'robotics.core.internal.CollisionGeometryBase'};
            obj.DataImportView.DataImportEventName = "CollisionFileAdded";
            obj.DataImportView.WindowName = obj.ADDCOLLISIONWINDOWTITLE;
            obj.DataImportView.TablePanelText = obj.COLLISIONVARSTABLELABEL;
            obj.DataImportView.OKButtonText = obj.ADDBUTTONLABEL;
            obj.DataImportView.HelpDocID = obj.ADDCOLLISIONDOCID;

            % Open the DataImportView dialog
            obj.DataImportView.createUIFromWorkSpace();
        end

        function loadSolver(obj)
        %loadSolver Load the dialog to import an existing IK solver

            obj.initializeWindowForModalUI;

            obj.DataImportView.DescriptionText = obj.IMPORTSOLVERINSTRUCTIONTEXT;
            obj.DataImportView.DataTypeFilter = {'inverseKinematics', 'generalizedInverseKinematics'};
            obj.DataImportView.DataImportEventName = "SolverAdded";
            obj.DataImportView.WindowName = obj.IMPORTSOLVERWINDOWTITLE;
            obj.DataImportView.TablePanelText = obj.SOLVERVARSTABLELABEL;
            obj.DataImportView.OKButtonText = obj.IMPORTBUTTONLABEL;
            obj.DataImportView.HelpDocID = obj.IMPORTSOLVERDOCID;

            % Open the DataImportView dialog
            obj.DataImportView.createUIFromWorkSpace();
        end

        function loadConstraints(obj)
        %loadConstraints Load the dialog to import existing IK constraints

            obj.initializeWindowForModalUI;

            obj.DataImportView.DescriptionText = obj.IMPORTCONSTRAINTSINSTRUCTIONTEXT;
            obj.DataImportView.DataTypeFilter = {'constraintAiming', 'constraintPoseTarget', 'constraintCartesianBounds', 'constraintJointBounds'};
            obj.DataImportView.DataImportEventName = "ConstraintsAdded";
            obj.DataImportView.WindowName = obj.IMPORTCONSTRAINTSWINDOWTITLE;
            obj.DataImportView.TablePanelText = obj.CONSTRAINTSVARSTABLELABEL;
            obj.DataImportView.OKButtonText = obj.IMPORTBUTTONLABEL;
            obj.DataImportView.HelpDocID = obj.IMPORTCONSTRAINTSDOCID;

            % Open the DataImportView dialog
            obj.DataImportView.createUIFromWorkSpace();
        end

        function loadConfigurations(obj)
        %loadConfigurations Load the dialog to import existing configurations

            obj.initializeWindowForModalUI;

            obj.DataImportView.DescriptionText = obj.IMPORTCONFIGSINSTRUCTIONTEXT;
            obj.DataImportView.DataTypeFilter = {'double'};
            obj.DataImportView.DataImportEventName = "ConfigurationsAdded";
            obj.DataImportView.WindowName = obj.IMPORTCONFIGSWINDOWTITLE;
            obj.DataImportView.TablePanelText = obj.CONFIGSVARSTABLELABEL;
            obj.DataImportView.OKButtonText = obj.IMPORTBUTTONLABEL;
            obj.DataImportView.HelpDocID = obj.IMPORTCONFIGSDOCID;

            % Open the DataImportView dialog
            obj.DataImportView.createUIFromWorkSpace();
        end

        function initializeWindowForModalUI(obj)
            %initializeWindowForModalUI Change app window and toolstrip status given modal UI request
            %   When a modal UI figure (a window that prevents interaction
            %   with the rest of the app) is opened, some states on the
            %   toolstrip and app window are changed to indicate to the
            %   user that interaction is blocked.

                obj.sendNotification("RequestAppBusy",[]);

        end
    end

    % Methods for building the toolstrip
    methods (Access = private)
        function addFileSection(obj)
        %addFileSection Add File section to toolstrip

        % Create the file section
            obj.FileSection = matlab.ui.internal.toolstrip.Section(obj.FILESECTIONLABEL);
            obj.FileSection.Tag = "FileSection";
            obj.TabHandle.add(obj.FileSection);

            % Add the New Session button
            col = obj.FileSection.addColumn();
            obj.NewSessionButton = matlab.ui.internal.toolstrip.Button(obj.NEWSESSIONBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.NEW_24);
            obj.NewSessionButton.Tag = "NewSessionButton";
            obj.NewSessionButton.Description = obj.NEWSESSIONBUTTONTOOLTIP;
            obj.NewSessionButton.ButtonPushedFcn = @(src,evt)obj.loadRigidBodyTree();
            col.add(obj.NewSessionButton);

            % Add the Open Session button
            col = obj.FileSection.addColumn();
            obj.OpenSessionButton = matlab.ui.internal.toolstrip.Button(obj.OPENSESSIONBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.OPEN_24);
            obj.OpenSessionButton.Tag = "OpenSessionButton";
            obj.OpenSessionButton.Description = obj.OPENSESSIONBUTTONTOOLTIP;
            obj.OpenSessionButton.ButtonPushedFcn = @(src,evt)obj.initiateLoadSession();
            col.add(obj.OpenSessionButton);

            % Add the button that allows us to save sessions. This is a
            % toggle-split button, meaning there is a main button and then
            % a list of sub-buttons. The main button is added first. The
            % button then contains a list of smaller buttons the user can
            % click on. These are attached with a helper method that
            % accepts a cell array. For each input, the cell array contains
            % the button name, icon, tag, and callback.
            col = obj.FileSection.addColumn();
            buttonName = obj.SAVESESSIONBUTTONLABEL;
            obj.SaveButton = matlab.ui.internal.toolstrip.SplitButton(buttonName, matlab.ui.internal.toolstrip.Icon.SAVE_24);
            obj.SaveButton.Tag = "SaveSessionButton";
            obj.SaveButton.Description = obj.SAVESESSIONBUTTONTOOLTIP;
            obj.SaveButton.Popup = matlab.ui.internal.toolstrip.PopupList();
            obj.SaveButton.ButtonPushedFcn = @(src,evt)obj.initiateSaveSession();

            popupListButtonsCellArray = {...
                {obj.SAVEBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.SAVE_16, "SaveButton", @(src,evt)obj.initiateSaveSession(), string.empty}, ...
                {obj.SAVEASBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.SAVE_AS_16, "SaveAsButton", @(src,evt)obj.initiateSaveAsSession(), string.empty}, ...
                   };
            obj.populatePopupList(obj.SaveButton, popupListButtonsCellArray);

            % Add the whole button to the column
            col.add(obj.SaveButton);

            % Add the button that allows us to import solvers, constraints,
            % and configurations. This is a toggle-split button, meaning
            % there is a main button and then a list of sub-buttons. The
            % main button is added first. The button then contains a list
            % of smaller buttons the user can click on. These are attached
            % with a helper method that accepts a cell array. For each
            % input, the cell array contains the button name, icon, tag,
            % and callback.
            col = obj.FileSection.addColumn();
            buttonName = obj.IMPORTBUTTONLABEL;
            button = matlab.ui.internal.toolstrip.SplitButton(buttonName, matlab.ui.internal.toolstrip.Icon.IMPORT_24);
            button.Tag = "ImportButton";
            button.Description = obj.IMPORTBUTTONTOOLTIP;
            button.Popup = matlab.ui.internal.toolstrip.PopupList();
            button.ButtonPushedFcn = @(src,evt)obj.loadSolver();

            popupListButtonsCellArray = {...
                {obj.IMPORTSOLVERBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.IMPORT_16, "ImportSolverButton", @(src,evt)obj.loadSolver(), obj.IMPORTSOLVERBUTTONTOOLTIP}, ...
                {obj.IMPORTCONSTRAINTSBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.IMPORT_16, "ImportConstraintsButton", @(src,evt)obj.loadConstraints(), obj.IMPORTCONSTRAINTSBUTTONTOOLTIP}, ...
                {obj.IMPORTCONFIGSBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.IMPORT_16, "ImportConfigurationsButton", @(src,evt)obj.loadConfigurations(), obj.IMPORTCONFIGSBUTTONTOOLTIP}};
            obj.populatePopupList(button, popupListButtonsCellArray);

            % Add the whole button to the column
            col.add(button);
        end

        function addSceneSection(obj)
        %addSceneSection Add scene section to toolstrip

            obj.SceneSection = matlab.ui.internal.toolstrip.Section(obj.SCENESECTIONLABEL);
            obj.SceneSection.Tag = "SceneSection";
            obj.TabHandle.add(obj.SceneSection);

            % Add the Add Collision Object button
            col = obj.SceneSection.addColumn();
            buttonName = obj.ADDCOLLISIONBUTTONLABEL;
            button = matlab.ui.internal.toolstrip.Button(buttonName, obj.ADDCOLLISIONICON);
            button.Tag = "AddCollisionButton";
            button.Description = obj.ADDCOLLISIONBUTTONTOOLTIP;
            button.ButtonPushedFcn = @(~,~)obj.loadCollisionObject();
            col.add(button);
        end

        function addIKSection(obj)
        %addIKSection Add the inverse kinematics section to the main toolstrip

            obj.IKSection = matlab.ui.internal.toolstrip.Section(obj.IKSECTIONLABEL);
            obj.IKSection.Tag = "IKSection";
            obj.TabHandle.add(obj.IKSection);

            % Add the New Constraint button
            col = obj.IKSection.addColumn();
            button = matlab.ui.internal.toolstrip.Button(obj.ADDCONSTRAINTBUTTONLABEL, obj.ADDCONSTRAINTICON);
            button.Tag = "AddConstraintButton";
            button.Description = obj.ADDCONSTRAINTBUTTONTOOLTIP;
            button.ButtonPushedFcn = @(~,~)obj.sendNotification("RequestAddConstraint", []);
            col.add(button);

            % Add the Edit Constraint button
            col = obj.IKSection.addColumn();
            obj.EditConstraintButton = matlab.ui.internal.toolstrip.Button(obj.EDITCONSTRAINTBUTTONLABEL, obj.EDITCONSTRAINTICON);
            obj.EditConstraintButton.Tag = "EditConstraintButton";
            obj.EditConstraintButton.ButtonPushedFcn = @(~,~)obj.sendNotification("RequestEditConstraint", []);
            obj.EditConstraintButton.Description = obj.EDITCONSTRAINTBUTTONTOOLTIP;
            col.add(obj.EditConstraintButton);

            % Create a column with solver options buttons
            col = obj.IKSection.addColumn();

            % Refresh solver
            button = matlab.ui.internal.toolstrip.Button(obj.REFRESHSOLVERBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.REFRESH_16);
            button.Tag = "SolverRefreshButton";
            button.Description = obj.REFRESHSOLVERBUTTONTOOLTIP;
            button.ButtonPushedFcn = @(src,evt)obj.notifySolverRefresh();
            col.add(button);

            % Solver settings
            button = matlab.ui.internal.toolstrip.Button(obj.SOLVERSETTINGSBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.SETTINGS_16);
            button.Tag = "SolverSettingsButton";
            button.Description = obj.SOLVERSETTINGSBUTTONTOOLTIP;
            button.ButtonPushedFcn = @(src,evt)obj.updateToolstripTabDisplay({}, {}, robotics.ikdesigner.internal.constants.Toolstrip.SolverTabTag, false);
            col.add(button);

            % Solver report
            button = matlab.ui.internal.toolstrip.Button(obj.REPORTSTATUSBUTTONLABEL, obj.SOLVERREPORTICON);
            button.Tag = "ReportStatusButton";
            button.ButtonPushedFcn = @(src,evt)obj.SolverInfoView.show();
            button.Description = obj.REPORTSTATUSBUTTONTOOLTIP;
            col.add(button);
        end

        function addMarkerConstraintSection(obj)
        %addMarkerConstraintSection Add the marker constraint section to the main toolstrip

            obj.MarkerConstraintSection = matlab.ui.internal.toolstrip.Section(obj.MARKERSECTIONLABEL);
            obj.MarkerConstraintSection.Tag = "MarkerConstraintSection";
            obj.TabHandle.add(obj.MarkerConstraintSection);

            % Add the button that allows us to view the pose target
            col = obj.MarkerConstraintSection.addColumn();
            markerPoseConstraintButton = matlab.ui.internal.toolstrip.Button(obj.MARKERBUTTONLABEL, obj.POSECONSTRAINTICON);
            markerPoseConstraintButton.Tag = "PoseTargetButton";
            markerPoseConstraintButton.Description = obj.MARKERBUTTONTOOLTIP;
            markerPoseConstraintButton.ButtonPushedFcn = @(~,~)obj.sendNotification("RequestEditMarkerPoseConstraint", []);
            col.add(markerPoseConstraintButton);

            % Add a column to select the marker body
            col = obj.MarkerConstraintSection.addColumn();

            % Create a label for the dropdown
            label = matlab.ui.internal.toolstrip.Label(obj.MARKERBODYLABEL);
            label.Tag = "MarkerBodyDropdownLabel";
            label.Description = obj.MARKERBODYTOOLTIP;
            col.add(label);

            % Create the marker body dropdown. This list is unpopulated at
            % tab construction, but will be populated with marker body
            % names during setup.
            obj.MarkerBodyDropdown = matlab.ui.internal.toolstrip.DropDown({' '});
            obj.MarkerBodyDropdown.Tag = "MarkerBodyDropDown";
            obj.MarkerBodyDropdown.Description = obj.MARKERBODYTOOLTIP;
            obj.MarkerBodyDropdown.Value = ' ';
            obj.MarkerBodyDropdown.ValueChangedFcn = @(src,evt)obj.notifyMarkerBodySelection(evt);
            col.add(obj.MarkerBodyDropdown);

            % Add a placeholder in the last row to align the other
            % entries correctly
            obj.addColumnPlaceholderLabel(col, "MarkerBodyPlaceholderLabel");
        end

        function addCollisionsSection(obj)
        %addCollisionsSection Add collisions section to toolstrip

            obj.CollisionsSection = matlab.ui.internal.toolstrip.Section(obj.COLLISIONSSECTIONLABEL);
            obj.CollisionsSection.Tag = "CollisionSection";
            obj.TabHandle.add(obj.CollisionsSection);

            % Add the button that allows us to check collisions. This is a
            % toggle-split button, meaning there is a main button and then
            % a list of sub-buttons. The main button is added first. The
            % button then contains a list of smaller buttons the user can
            % click on. These are attached with a helper method that
            % accepts a cell array. For each input, the cell array contains
            % the button name, icon, tag, and callback.
            col = obj.CollisionsSection.addColumn();
            button = matlab.ui.internal.toolstrip.SplitButton(obj.CHECKCOLLISIONSBUTTONLABEL, obj.CHECKCOLLISIONICON);
            button.Tag = "CheckCollisionsButton";
            button.Description = obj.CHECKCOLLISIONSBUTTONTOOLTIP;
            button.Popup = matlab.ui.internal.toolstrip.PopupList();
            button.ButtonPushedFcn = @(src,evt)obj.notifyCheckCollision(1);

            popupListButtonsCellArray = {...
                {obj.CHECKCURRCOLLISIONSBUTTONLABEL, obj.CHECKCOLLISIONICON, "CheckCurrentConfigurationButton", @(src,evt)obj.notifyCheckCollision(1), obj.CHECKCURRCOLLISIONSBUTTONTOOLTIP}, ...
                {obj.CHECKALLCOLLISIONSBUTTONLABEL, obj.CHECKCOLLISIONICON, "CheckAllConfigurationsButton", @(src,evt)obj.notifyCheckCollision(2), obj.CHECKALLCOLLISIONSBUTTONTOOLTIP}};
            obj.populatePopupList(button, popupListButtonsCellArray);

            % Add a separator to split the actions from the options
            checkboxHeader = matlab.ui.internal.toolstrip.PopupListHeader(obj.COLLISIONOPTSLABEL);
            button.Popup.add(checkboxHeader);

            % Add a checkbox for self-collision checks to this popup
            listButtonItem = matlab.ui.internal.toolstrip.ListItemWithCheckBox(obj.IGNORESELFCOLLISIONCHECKBOXLABEL);
            listButtonItem.Tag = "CheckSelfCollisionCheckbox";
            listButtonItem.ValueChangedFcn = @(src,evt)obj.enableIgnoreSelfCollisions(src, evt);
            listButtonItem.Description = obj.IGNORESELFCOLLISIONCHECKBOXTOOLTIP;
            button.Popup.add(listButtonItem);

            % Store the handle to the checkbox
            obj.IgnoreSelfCollisionsCheckbox = listButtonItem;

            % Add the whole button to the column
            col.add(button);
        end

        function addExportSection(obj)
        %addExportSection Add the Export section to the toolstrip

            obj.ExportSection = matlab.ui.internal.toolstrip.Section(obj.EXPORTSECTIONLABEL);
            obj.ExportSection.Tag = "ExportSection";
            obj.TabHandle.add(obj.ExportSection);

            % Add the button that allows us to export data. This is a
            % toggle-split button, meaning there is a main button and then
            % a list of sub-buttons. The main button is added first. The
            % button then contains a list of smaller buttons the user can
            % click on. These are attached with a helper method that
            % accepts a cell array. For each input, the cell array contains
            % the button name, icon, tag, and callback.
            col = obj.ExportSection.addColumn();
            button = matlab.ui.internal.toolstrip.SplitButton(obj.EXPORTBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.CONFIRM_24);
            button.Tag = "ExportButton";
            button.Description = obj.EXPORTBUTTONTOOLTIP;
            button.Popup = matlab.ui.internal.toolstrip.PopupList();
            button.ButtonPushedFcn = @(src,evt)obj.sendNotification("RequestSolverDataExport", []);

            popupListButtonsCellArray = {...
                {obj.EXPORTSOLVERBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.EXPORT_16, "ExportSolverAndConstraintsButton", @(src,evt)obj.sendNotification("RequestSolverDataExport", []), obj.EXPORTSOLVERBUTTONTOOLTIP}, ...
                {obj.EXPORTCONFIGSBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.EXPORT_16, "ExportConfigurationsButton", @(src,evt)obj.sendNotification("RequestConfigDataExport", []), obj.EXPORTCONFIGSBUTTONTOOLTIP}};
            obj.populatePopupList(button, popupListButtonsCellArray);

            % Add the whole button to the column
            col.add(button);
        end
    end

    methods (Static, Access = private)
        function [dropdownListText, dropdownListCallbacks] = getRobotList()
        %getRobotList Get the dropdown list of robots in the robot library
        %   This method returns the dropdown list of qualifying robots
        %   in the robot library. The list assembles the manipulator
        %   and mobile manipulator rigid body trees in the library,
        %   along with the specified names. The actual list uses the
        %   names as the items, and then populates the ItemsData part
        %   with a cell array that contains the handles to the callback
        %   and to a function that generates the displayed name.

        % The list of robots is an alphabetically sorted list of robots
        % that are classified as manipulators or mobile manipulators
            availableRobotNames = [robotics.manip.internal.RobotList.ManipulatorRobotNames robotics.manip.internal.RobotList.MobileManipulatorRobotNames];
            availableRobotFcnInputs = [robotics.manip.internal.RobotList.ManipulatorRobots robotics.manip.internal.RobotList.MobileManipulatorRobots];
            [sortedRobotNames, idx] = sort(availableRobotNames);
            sortedRobotFcnInputs = availableRobotFcnInputs(idx);

            % The list has two values: user-facing text and callbacks. The
            % callbacks are presented here as a two-element cell array,
            % where the first is a callback that loads the data, and the
            % second is a callback that loads the variable that will be
            % used to display the data in the app. Here, "robot" is always
            % used.
            dropdownListText = sortedRobotNames;
            dropdownListCallbacks = cell(1, numel(dropdownListText));
            for i = 1:numel(dropdownListCallbacks)
                dropdownListCallbacks{i} = {...
                    @()({loadrobot(sortedRobotFcnInputs(i), "DataFormat","column")}) ...
                    @(obj)({'Robot'}) ...
                   };
            end
        end
    end

    methods (Static, Access = ?robotics.ikdesigner.internal.view.View)
        function validateImportedTreeData(dataCellArray)
            %validateImportedTreeData Verify that robot selected by user is valid for use in IK app

            validateattributes(dataCellArray, {'cell'}, {'numel', 1}, 'inverseKinematicsDesigner', 'Selected rigidbodytree');
            
            robot = dataCellArray{1};
            validateattributes(robot, {'rigidBodyTree'}, {'nonempty','scalar'}, 'inverseKinematicsDesigner', 'rigidBodyTree');
            robotics.manip.internal.RigidBodyTreeUtils.validateTreeWithMovingJoints(robot);
        end

    end
end
