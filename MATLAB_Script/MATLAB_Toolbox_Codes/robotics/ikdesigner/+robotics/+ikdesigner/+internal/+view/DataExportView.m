classdef DataExportView < robotics.ikdesigner.internal.view.CustomDataUI
%This function is for internal use only and may be removed in a future release

%DataExportView Views for handling data export to the workspace

%   Copyright 2021 The MathWorks, Inc.

    properties (Access = {?matlab.unittest.TestCase, ?robotics_tests.app.ikdesigner.DataExportWindowTester})

        %SolverNameField Edit field that allows user to specify name for the exported solver
        SolverNameField

        %ConstraintsNameField Edit field that allows user to specify name for the exported constraints cell array
        ConstraintsNameField

        %ConfigsNameField Edit field that allows user to specify name for the exported configurations matrix
        ConfigsNameField

        %SolverActivationCheckbox Checkbox that indicates whether or not to export the solver
        SolverActivationCheckbox

        %ConstraintsActivationCheckbox Checkbox that indicates whether or not to export the constraints cell array
        ConstraintsActivationCheckbox

        %WSExportFcn Function that is called to export data to the workspace
        WSExportFcn

    end

    properties (Constant, Access = protected)
        %Available (exportable) constraints panel label
        AVAILABLECONSTRAINTSLABEL = string(message('robotics:ikdesigner:dataimportexport:AvailableConstraintsLabel'))

        %Available (exportable) configurations panel label
        AVAILABLECONFIGSLABEL = string(message('robotics:ikdesigner:dataimportexport:AvailableConfigurationsLabel'))

        %Waypoints variable name label and tooltip
        WAYPOINTSNAMELABEL = string(message('robotics:ikdesigner:dataimportexport:WaypointMatrixNameLabel'))

        %Export solver checkbox label and tooltip
        EXPORTSOLVERLABEL = string(message('robotics:ikdesigner:dataimportexport:ExportSolverLabel'))
        EXPORTSOLVERTOOLTIP = string(message('robotics:ikdesigner:dataimportexport:ExportSolverTooltip'))

        %Export constraints checkbox label and tooltip
        EXPORTCONSTRAINTSLABEL = string(message('robotics:ikdesigner:dataimportexport:ExportConstraintsLabel'))
        EXPORTCONSTRAINTSTOOLTIP = string(message('robotics:ikdesigner:dataimportexport:ExportConstraintsTooltip'))

        %Solver variable name label and tooltip
        SOLVERNAMELABEL = string(message('robotics:ikdesigner:dataimportexport:SolverNameLabel'))
        SOLVERNAMETOOLTIP = string(message('robotics:ikdesigner:dataimportexport:SolverNameTooltip'))

        %Constraints variable name label and tooltip
        CONSTRAINTSNAMELABEL = string(message('robotics:ikdesigner:dataimportexport:ConstraintsArrayNameLabel'))
        CONSTRAINTSNAMETOOLTIP = string(message('robotics:ikdesigner:dataimportexport:ConstraintsArrayNameTooltip'))

        %UIEXPORTBUTTONLABEL Export button label
        UIEXPORTBUTTONLABEL = string(message('robotics:ikdesigner:dataimportexport:UIExportButtonLabel'))
    end

    methods
        function obj = DataExportView(appWindow, parentComponent)
        %DataExportView Constructor

            obj@robotics.ikdesigner.internal.view.CustomDataUI(appWindow, parentComponent);
        end
    end

    methods (Access = {?robotics.ikdesigner.internal.view.View, ?robotics.ikdesigner.internal.toolstrip.Tab, ?matlab.unittest.TestCase})

        function createSolverConstraintsExportView(obj, solver, constraintArray, constraintNamesArray)
        %createSolverConstraintsExportView Create UI for exporting the solver and constraints to the workspace

            if obj.IsWindowOpen
                return;
            end
            figGrid = obj.createBaselineUI();

            obj.TablePanelText = obj.AVAILABLECONSTRAINTSLABEL;
            obj.OKButtonText = obj.UIEXPORTBUTTONLABEL;
            obj.TableVariables = cellstr(constraintNamesArray);
            obj.TableVariableClasses = cellfun(@(x)(class(x)), constraintArray, 'UniformOutput',false);
            obj.TableVariableSizes = cellfun(@(x)(num2str(size(x))), constraintArray, 'UniformOutput',false);

            % Build all the components
            obj.buildDescriptionArea(figGrid);
            [obj.SolverNameField, obj.SolverActivationCheckbox] = obj.buildNameField(figGrid, obj.EXPORTSOLVERLABEL, obj.SOLVERNAMELABEL, "ikSolver");
            [obj.ConstraintsNameField, obj.ConstraintsActivationCheckbox] = obj.buildNameField(figGrid, obj.EXPORTCONSTRAINTSLABEL, obj.CONSTRAINTSNAMELABEL, "ikConstraints");
            obj.buildVariableSelectionPanel(figGrid, false);
            obj.buildAcceptanceButtons(figGrid);

            % Assign an action on OK button
            obj.WSExportFcn = @()obj.saveSolverAndConstraintsToWS(solver, constraintArray);

            % Assign callbacks to the checkboxes
            solverRefreshCB = @()(obj.refreshSolverConstraintsOKButton);
            obj.SolverActivationCheckbox.ValueChangedFcn = @(src,evt)obj.updateNameFieldActivation(src, obj.SolverNameField, solverRefreshCB);
            constraintRefreshCB = @()(obj.refreshConstraintsTableEnable);
            obj.ConstraintsActivationCheckbox.ValueChangedFcn = @(src,evt)obj.updateNameFieldActivation(src, obj.ConstraintsNameField, constraintRefreshCB);

            obj.refreshSolverConstraintsOKButton();
        end

        function createConfigExportView(obj, configDataArray, configNamesArray)
        %createConfigExportView Create UI for exporting the configurations to the workspace

            if obj.IsWindowOpen
                return;
            end
            
            figGrid = obj.createBaselineUI();

            obj.TablePanelText = obj.AVAILABLECONFIGSLABEL;
            obj.OKButtonText = obj.UIEXPORTBUTTONLABEL;
            obj.TableVariables = cellstr(configNamesArray);
            obj.TableVariableClasses = cellfun(@(x)(class(x)), configDataArray, 'UniformOutput',false);
            obj.TableVariableSizes = cellfun(@(x)(num2str(size(x))), configDataArray, 'UniformOutput',false);

            % Build all the components
            obj.buildDescriptionArea(figGrid);
            obj.ConfigsNameField = obj.buildNameField(figGrid, string.empty, obj.WAYPOINTSNAMELABEL, "waypointData");
            obj.buildVariableSelectionPanel(figGrid, false);
            obj.buildAcceptanceButtons(figGrid);

            % Assign an action on OK button
            obj.WSExportFcn = @()obj.saveConfigurationsToWS(configDataArray);

            obj.refreshOKButtonEnableState();
        end
    end

    methods (Access = protected)
        function actOnOKButtonSelection(obj)
        %actOnOKButtonSelection Take this action when the OK button is clicked

            obj.WSExportFcn();
        end

        function refreshSolverConstraintsOKButton(obj)
        %refreshOKButtonEnableState Refresh action when solver constraint export is enabled/disabled

            obj.refreshOKButtonEnableState;

            if obj.SolverActivationCheckbox.Value || obj.ConstraintsActivationCheckbox.Value
                obj.OKButton.Enable = 'on';
            end

            if ~obj.SolverActivationCheckbox.Value && ~obj.ConstraintsActivationCheckbox.Value
                obj.OKButton.Enable = 'off';
            end
        end

        function refreshConstraintsTableEnable(obj)
        %refreshConstraintsTableEnable Refresh action when constraint export is enabled/disabled

            if obj.ConstraintsActivationCheckbox.Value
                obj.VariableTable.Enable = 'on';
            else
                obj.VariableTable.Enable = 'off';
            end

            obj.refreshSolverConstraintsOKButton();

        end
    end

    methods (Access = private)
        function importedData = exportSelectedWorkspaceData(obj)
        %loadSelectedWorkspaceData Load selected data from workspace
        %   Function to load get the values of data imported from the
        %   workspace. The method takes the cell array of selected
        %   variables in the workspace table and imports them, storing
        %   the values in a cell array.

            importedData = cellfun(@(x)evalin('base', x), obj.SelectedData, 'UniformOutput',false);
        end

        function [nameField, activationCheckbox] = buildNameField(obj, figGrid, checkboxText, labelText, defaultValueText)
        %buildDescriptionArea Add a description to the uifigure
        %   Add a uilabel containing a description of the uifigure to
        %   instruct the user how to use the UI.

        % Add a row to the grid
            rowNum = numel(figGrid.RowHeight) + 1;
            figGrid.RowHeight{rowNum} = 'fit';

            localGrid = uigridlayout(figGrid);
            localGrid.RowHeight = {'fit'};
            localGrid.ColumnWidth = {'1x', '1x', 100};
            localGrid.Layout.Row = rowNum;
            localGrid.Layout.Column = [1 4];
            localGrid.RowSpacing = 0;
            localGrid.Padding = [0 0 0 0];

            %Add a check box and label
            activationCheckbox = uicheckbox(localGrid);
            activationCheckbox.Layout.Row = 1;
            activationCheckbox.Layout.Column = 1;
            activationCheckbox.Value = 1;
            if isempty(checkboxText)
                activationCheckbox.Visible = 'off';
            else
                activationCheckbox.Visible = 'on';
                activationCheckbox.Text = checkboxText;
            end

            %Add a label
            nameLabel = uilabel(localGrid);
            nameLabel.Layout.Row = 1;
            nameLabel.Layout.Column = 2;
            nameLabel.Text = labelText;
            nameLabel.WordWrap = 'on';
            nameLabel.HorizontalAlignment = 'right';

            %Add an edit field
            nameField = uieditfield(localGrid, 'text');
            nameField.Layout.Row = 1;
            nameField.Layout.Column = 3;
            nameField.Value = defaultValueText;

            % Update the figure height
            obj.UIFigure.Position(4) = obj.UIFigure.Position(4) + localGrid.RowSpacing + nameField.Position(4);
        end

        function saveSolverAndConstraintsToWS(obj, solver, constraintArray)
        %saveSolverAndConstraintsToWS Assign solver and constraints to variables in the workspace

        % If the constraints are selected, get the indices of selected
        % constraints
            if obj.ConstraintsActivationCheckbox.Value
                constraintIndicesToExport = obj.SelectedTableIndices;
                constraintsToExport = constraintArray(constraintIndicesToExport);
                assignin('base', obj.ConstraintsNameField.Value, constraintsToExport);
            end

            % Update the solver so it works with the selected
            % constraints, then export
            if ~obj.ConstraintsActivationCheckbox.Value || isempty(constraintIndicesToExport)
                solver.ConstraintInputs = cell(1,0);
            else
                solver.ConstraintInputs = solver.ConstraintInputs(constraintIndicesToExport);
            end

            % Export the solver, if requested
            if obj.SolverActivationCheckbox.Value
                assignin('base', obj.SolverNameField.Value, solver);
            end

            obj.sendNotification("RequestAppNotBusy",[]);

        end

        function saveConfigurationsToWS(obj, configurationsArray)
        %saveConfigurationsToWS Assign configurations to a matrix in the workspace

        % Get the indices of selected configurations and store in a
        % matrix
            configIndicesToExport = obj.SelectedTableIndices;
            configsToExport = cell2mat(configurationsArray(configIndicesToExport))';
            assignin('base', obj.ConfigsNameField.Value, configsToExport);

            obj.sendNotification("RequestAppNotBusy",[]);

        end

    end

    methods (Static, Access = private)

        function updateNameFieldActivation(src, nameField, refreshCB)
        %updateNameFieldActivation Update the enabled state of a uieditfield given a change in the associated checkbox value

            if src.Value
                nameField.Enable = 'on';
            else
                nameField.Enable = 'off';
            end
            refreshCB();
        end
    end
end
