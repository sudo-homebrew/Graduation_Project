classdef SolverTab < robotics.ikdesigner.internal.toolstrip.Tab
%This class is for internal use only. It may be removed in the future.

%SolverTab Tab view that for the solver tab in the inverse kinematics designer toolstrip

%   Copyright 2021-2022 The MathWorks, Inc.

    properties (SetAccess = {?robotics.ikdesigner.internal.view.View, ?matlab.unittest.TestCase})

        % Algorithms section toolstrip elements
        AlgorithmsSection
        SolverAlgorithmDropdown
        MaxIterationsField
        MaxTimeField
        EnforceJointLimitsCheckbox
        AllowRandomRestartCheckbox

        % Options section toolstrip elements
        OptionsSection

        % Settings section toolstrip elements
        SettingsSection
        ResetSettingsButton
        ApplyToSolverButton
    end

    properties (SetAccess = private)

        %StoredSolverAlgorithm
        StoredSolverAlgorithm

        %StoredSolverParameters
        StoredSolverParameters

    end

    properties (Constant)
        DefaultSolverAlgorithm = "BFGSGradientProjection"

        DefaultSolverParameters = struct(...
            "MaxIterations", 1500, ...
            "MaxTime", 10, ...
            "EnforceJointLimits", true, ...
            "AllowRandomRestart", true)
    end

    properties (Constant, Access = private)
        %Solver algorithms section labels
        SOLVERALGSSECTIONTITLE = string(message('robotics:ikdesigner:toolstrip:SolverAlgorithmsSectionLabel'))
        SOLVERALGLABEL = string(message('robotics:ikdesigner:toolstrip:SolverAlgorithmLabel'))
        MAXITERATIONSLABEL = string(message('robotics:ikdesigner:toolstrip:MaxIterationsLabel'))
        MAXTIMELABEL = string(message('robotics:ikdesigner:toolstrip:MaxTimeLabel'))
        BFGSNAMELABEL = string(message('robotics:ikdesigner:toolstrip:BFGSLabel'))
        LMNAMELABEL = string(message('robotics:ikdesigner:toolstrip:LevenbergMarquardtLabel'))
        ENFORCEJTLIMSLABEL = string(message('robotics:ikdesigner:toolstrip:EnforceJointLimitsLabel'))
        ALLOWRANDRESTARTLABEL = string(message('robotics:ikdesigner:toolstrip:AllowRandomRestartsLabel'))

        %Solver algorithms section tooltips
        SOLVERALGTOOLTIP = string(message('robotics:ikdesigner:toolstrip:SolverAlgorithmTooltip'))
        MAXITERATIONSTOOLTIP = string(message('robotics:ikdesigner:toolstrip:MaxIterationsTooltip'))
        MAXTIMETOOLTIP = string(message('robotics:ikdesigner:toolstrip:MaxTimeTooltip'))
        ENFORCEJTLIMSTOOLTIP = string(message('robotics:ikdesigner:toolstrip:EnforceJointLimitsTooltip'))
        ALLOWRANDRESTARTTOOLTIP = string(message('robotics:ikdesigner:toolstrip:AllowRandomRestartsTooltip'))

        %Solver settings section labels
        SOLVERSETTINGSSECTIONTITLE = string(message('robotics:ikdesigner:toolstrip:SolverSettingsSectionLabel'))
        RESETSETTINGSPOPUPBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:ResetSettingsPopupButtonLabel'))
        RESETSETTINGSBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:ResetSettingsButtonLabel'))
        RESETTODEFAULTBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:ResetToDefaultButtonLabel'))
        APPLYBUTTONLABEL = string(message('robotics:ikdesigner:toolstrip:ApplyToSolverButtonLabel'))

        %Solver settings section tooltips
        RESETSETTINGSPOPUPBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:ResetSettingsButtonTooltip'))
        RESETSETTINGSBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:ResetSettingsButtonTooltip'))
        RESETTODEFAULTBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:ResetToDefaultButtonTooltip'))
        APPLYBUTTONTOOLTIP = string(message('robotics:ikdesigner:toolstrip:ApplyToSolverButtonTooltip'))
    end

    methods
        function obj = SolverTab(toolstrip)
        %SolverTab Constructor

        % Construct the tab and associate it with the toolstrip
            obj@robotics.ikdesigner.internal.toolstrip.Tab(toolstrip, "Solver", "SolverTab");

            % Add sections to the toolstrip
            obj.addAlgorithmsSection();
            obj.addSettingsSection();

            % Move tab to initial state when app is loaded
            obj.TabHandle.disableAll();
        end

        function setup(obj, ikSolver)
        %setup Set up the tab with new session data

            obj.populateSolverTab(ikSolver.SolverAlgorithm, ikSolver.SolverParameters);
            obj.storeSolverSettings;

        end

        function initialize(obj)

        % Enable all except the solver button
            obj.enableAll;
            obj.ApplyToSolverButton.Enabled = false;

        end

        function notifySolverSettingsUpdated(obj)
        %notifySolverSettingsUpdated Store the solver settings and send the notification that they have been changed

            obj.storeSolverSettings;
            eventData = robotics.ikdesigner.internal.event.SolverSettingsEventData(obj.StoredSolverAlgorithm, obj.StoredSolverParameters);
            obj.sendNotification('SolverSettingsEdited', eventData);

            % Since the button has been pushed, disable it
            obj.ApplyToSolverButton.Enabled = false;
        end

        function populateSolverTab(obj, solverAlgorithm, solverParameters)
        %populateSolverTab Populate the displayed solver settings from internally stored values

        % Assign solver algorithm
            obj.SolverAlgorithmDropdown.Value = solverAlgorithm;

            % Assign dependent parameters
            obj.MaxIterationsField.Value = num2str(solverParameters.MaxIterations);
            obj.MaxTimeField.Value = num2str(solverParameters.MaxTime);
            obj.EnforceJointLimitsCheckbox.Value = solverParameters.EnforceJointLimits;
            obj.AllowRandomRestartCheckbox.Value = solverParameters.AllowRandomRestart;

        end
    end

    methods (Access = private)
        function addAlgorithmsSection(obj)
        %addAlgorithmsSection Add the algorithms section to the tab

            obj.AlgorithmsSection = matlab.ui.internal.toolstrip.Section("Algorithms");
            obj.AlgorithmsSection.Tag = "AlgorithmsSection";
            obj.TabHandle.add(obj.AlgorithmsSection);

            % Create a column for the ui component labels
            col = obj.AlgorithmsSection.addColumn();

            % Add the solver algorithm label
            label = matlab.ui.internal.toolstrip.Label("Solver Algorithm");
            label.Tag = "SolverAlgorithm";
            label.Description = obj.SOLVERALGTOOLTIP;
            col.add(label);

            % Add the max iterations label
            label = matlab.ui.internal.toolstrip.Label("Max Iterations");
            label.Tag = "MaxIterations";
            label.Description = obj.MAXITERATIONSTOOLTIP;
            col.add(label);

            % Add the max time label
            label = matlab.ui.internal.toolstrip.Label("Max Time (s)");
            label.Tag = "MaxTime";
            label.Description = obj.MAXTIMETOOLTIP;
            col.add(label);

            % Create a column for the core settable solver parameters
            col = obj.AlgorithmsSection.addColumn();

            % Add the solver algorithms dropdown
            dropdownLabels = {'BFGS Gradient Projection'; 'Levenberg-Marquardt'};
            dropdownValues = {'BFGSGradientProjection'; 'LevenbergMarquardt'};
            obj.SolverAlgorithmDropdown = matlab.ui.internal.toolstrip.DropDown([dropdownValues dropdownLabels]);
            obj.SolverAlgorithmDropdown.Tag = "SolverAlgorithmDropdown";
            obj.SolverAlgorithmDropdown.Description = obj.SOLVERALGTOOLTIP;
            obj.SolverAlgorithmDropdown.Value = dropdownValues{1};
            obj.SolverAlgorithmDropdown.ValueChangedFcn = @(src,evt)obj.enableSolverEditedStatus();
            col.add(obj.SolverAlgorithmDropdown);

            % Add the Max Iterations edit field
            obj.MaxIterationsField = matlab.ui.internal.toolstrip.EditField('20');
            obj.MaxIterationsField.Tag = "MaxIterationsField";
            obj.MaxIterationsField.Description = obj.MAXITERATIONSTOOLTIP;
            obj.MaxIterationsField.ValueChangedFcn = @(src,evt)obj.editMaxIterations(src,evt);
            col.add(obj.MaxIterationsField);

            % Add the Max Time edit field
            obj.MaxTimeField = matlab.ui.internal.toolstrip.EditField('0.5');
            obj.MaxTimeField.Tag = "MaxTimeField";
            obj.MaxTimeField.Description = obj.MAXTIMETOOLTIP;
            obj.MaxTimeField.ValueChangedFcn = @(src,evt)obj.editMaxTime(src,evt);
            col.add(obj.MaxTimeField);

            % Add a column for boolean (checkbox) parameters
            col = obj.AlgorithmsSection.addColumn();

            % Add a placeholder in the first row to align the remaining
            % entries correctly
            obj.addColumnPlaceholderLabel(col, "MarkerBodyPlaceholderLabel");

            % Add the Enforce Joint Limits checkbox
            obj.EnforceJointLimitsCheckbox = matlab.ui.internal.toolstrip.CheckBox("Enforce Joint Limits");
            obj.EnforceJointLimitsCheckbox.Tag = "EnforceJointLimitsCheckbox";
            obj.EnforceJointLimitsCheckbox.Description = obj.ENFORCEJTLIMSTOOLTIP;
            obj.EnforceJointLimitsCheckbox.ValueChangedFcn = @(src,evt)obj.enableSolverEditedStatus();
            col.add(obj.EnforceJointLimitsCheckbox);

            % Add the Allow Random Restart checkbox
            obj.AllowRandomRestartCheckbox = matlab.ui.internal.toolstrip.CheckBox("Allow Random Restart");
            obj.AllowRandomRestartCheckbox.Tag = "AllowRandomRestartCheckbox";
            obj.AllowRandomRestartCheckbox.Description = obj.ALLOWRANDRESTARTTOOLTIP;
            obj.AllowRandomRestartCheckbox.ValueChangedFcn = @(src,evt)obj.enableSolverEditedStatus();
            col.add(obj.AllowRandomRestartCheckbox);

        end

        function addSettingsSection(obj)
        %addSettingsSection Add the settings section to the tab

            obj.SettingsSection = matlab.ui.internal.toolstrip.Section("Settings");
            obj.SettingsSection.Tag = "SettingsSection";
            obj.TabHandle.add(obj.SettingsSection);

            % Add the Reset Settings button, which is a dropdown to show
            % the different reset options. This is a toggle-split button,
            % meaning there is a main button and then a list of
            % sub-buttons. The main button is added first. The button then
            % contains a list of smaller buttons the user can click on.
            % These are attached with a helper method that accepts a cell
            % array. For each input, the cell array contains the button
            % name, icon, tag, and callback.
            col = obj.SettingsSection.addColumn();
            buttonName = obj.RESETSETTINGSPOPUPBUTTONLABEL;
            obj.ResetSettingsButton = matlab.ui.internal.toolstrip.SplitButton(buttonName, matlab.ui.internal.toolstrip.Icon.RESTORE_24);
            obj.ResetSettingsButton.Tag = "ResetSettingsBigButton";
            obj.ResetSettingsButton.Description = obj.RESETSETTINGSPOPUPBUTTONTOOLTIP;
            obj.ResetSettingsButton.ButtonPushedFcn = @(src,evt)obj.resetSolverSettingsToStoredValue();
            obj.ResetSettingsButton.Popup = matlab.ui.internal.toolstrip.PopupList();

            popupListButtonsCellArray = {...
                {obj.RESETSETTINGSBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.RESTORE_16, "ResetSettingsSmallButton",@(src,evt)obj.resetSolverSettingsToStoredValue(), obj.RESETSETTINGSBUTTONTOOLTIP}, ...
                {obj.RESETTODEFAULTBUTTONLABEL, matlab.ui.internal.toolstrip.Icon.RESTORE_16, "ResetToDefaultSettingsButton",@(src,evt)obj.resetSolverSettingsToDefaults(), obj.RESETTODEFAULTBUTTONTOOLTIP}};
            obj.populatePopupList(obj.ResetSettingsButton, popupListButtonsCellArray);
            col.add(obj.ResetSettingsButton);

            % Add the Apply button
            col = obj.SettingsSection.addColumn();
            buttonName = obj.APPLYBUTTONLABEL;
            obj.ApplyToSolverButton = matlab.ui.internal.toolstrip.Button(buttonName, matlab.ui.internal.toolstrip.Icon.CONFIRM_24);
            obj.ApplyToSolverButton.Tag = "ApplyToSolverButton";
            obj.ApplyToSolverButton.Description = obj.APPLYBUTTONTOOLTIP;
            obj.ApplyToSolverButton.ButtonPushedFcn = @(src,evt)obj.notifySolverSettingsUpdated();
            col.add(obj.ApplyToSolverButton);
        end
    end

    methods (Access = private)
        function storeSolverSettings(obj)
        %storeSolverSettings Store displayed solver settings

            obj.StoredSolverAlgorithm = obj.SolverAlgorithmDropdown.Value;

            obj.StoredSolverParameters = struct(...
                'MaxIterations', str2num(obj.MaxIterationsField.Value), ...
                'MaxTime', str2num(obj.MaxTimeField.Value), ...
                'EnforceJointLimits', obj.EnforceJointLimitsCheckbox.Value, ...
                'AllowRandomRestart', obj.AllowRandomRestartCheckbox.Value); %#ok<ST2NM> 
        end

        function enableSolverEditedStatus(obj)

            obj.ApplyToSolverButton.Enabled = true;

        end

        function resetSolverSettingsToStoredValue(obj)
        %resetSolverSettingsToStoredValue Reset solver settings to last stored values

            obj.populateSolverTab(obj.StoredSolverAlgorithm, obj.StoredSolverParameters);

            % Since this resets the values to what they were before the
            % user edited (but also before they applied), resetting should
            % disable the ability to apply.
            obj.ApplyToSolverButton.Enabled = false;
        end

        function resetSolverSettingsToDefaults(obj)
        %resetSolverSettingsToDefaults Reset solver settings to default values

            solverAlgorithm = robotics.ikdesigner.internal.constants.Data.DEFAULTSOLVERALGORITHM;
            solverParams = robotics.ikdesigner.internal.constants.Data.DEFAULTSOLVERPARAMS;
            obj.populateSolverTab(solverAlgorithm, solverParams);

            % Allow the user to apply these change
            obj.enableSolverEditedStatus();

        end

        function editMaxTime(obj, src, evt)

            fieldAttributes = {'nonempty', 'nonnan', 'finite', 'real', 'scalar', '>', 0};
            isValid = obj.validateAndSetNumericInput(src, evt, obj.MaxTimeField, obj.MAXTIMELABEL, fieldAttributes);
            if isValid
                obj.enableSolverEditedStatus();
            end
        end

        function editMaxIterations(obj, src, evt)

            fieldAttributes = {'nonempty',  'nonnan', 'finite', 'integer', 'scalar', '>',1};
            isValid = obj.validateAndSetNumericInput(src, evt, obj.MaxIterationsField, obj.MAXITERATIONSLABEL, fieldAttributes);
            if isValid
                obj.enableSolverEditedStatus();
            end
        end
    end

end
