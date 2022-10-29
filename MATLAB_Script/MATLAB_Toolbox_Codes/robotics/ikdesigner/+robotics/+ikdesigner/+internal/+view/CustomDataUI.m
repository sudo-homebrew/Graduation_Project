classdef CustomDataUI < robotics.ikdesigner.internal.view.View
%This function is for internal use only and may be removed in a future release

%CustomDataUI Views for handling data i/o in the app

%   Copyright 2021 The MathWorks, Inc.

    properties

        %UserCancelled Value to indicate that the user aborted the import
        UserCancelled logical

        %ValidDataSelection Flag to indicate whether data has been imported
        ValidDataSelection logical

        %DescriptionText Text displayed above the loading window
        DescriptionText

        %WindowName Name displayed on the uifigure
        WindowName

        %TablePanelText Text that titles the contents of the variable table panel
        TablePanelText

        %OKButtonText Text on the button that accepts the dialog
        OKButtonText

        %HelpDocID Doc ID that the help button points to
        %   By default, this just goes to the general inverse kinematics
        %   doc, but ideally each UI should reach a more specific target
        HelpDocID = "inverseKinematicsDesigner"
    end

    properties (Access = {?robotics.ikdesigner.internal.view.View, ...
            ?matlab.unittest.TestCase, ...
            ?robotics_tests.app.ikdesigner.DataImportWindowTester, ...
            ?robotics_tests.app.ikdesigner.DataExportWindowTester})

        %UIFigure Dialog uifigure
        UIFigure

        %TableRefreshButton Button to refresh the selection table
        TableRefreshButton

        %CancelButton Dialog cancel button
        CancelButton

        %OKButton Dialog OK button
        OKButton

        %VariableTable Table of variables
        VariableTable

        %TableVariables Cell array of variable names displayed in the table
        TableVariables

        %TableVariableSizes Cell array of sizes of displayed variables
        TableVariableSizes

        %TableVariableClasses Cell array of classes of displayed variables
        TableVariableClasses

        %SelectedTableIndices Indices of the variables selected in the table
        SelectedTableIndices

        %AppWindow App window containing the parent app
        AppWindow

        %Parent Parent object to whom the event requests are forwarded
        Parent
    end

    properties (Access = protected)
        %IsWindowOpen Property to check whether window is open
        %   This boolean ensures that at most one modal UI window can be
        %   open at a time.
        IsWindowOpen
    end

    properties (Dependent)
        %SelectedTableData Cell array of selected variable names
        SelectedTableData
    end

    properties (Constant, Access = protected)
        VARCOLLABEL = string(message('robotics:ikdesigner:dataimportexport:VariableColHeaderLabel'))

        SIZECOLLABEL = string(message('robotics:ikdesigner:dataimportexport:SizeColHeaderLabel'))

        CLASSCOLLABEL = string(message('robotics:ikdesigner:dataimportexport:ClassColHeaderLabel'))

        REFRESHBUTTONLABEL = string(message('robotics:ikdesigner:dataimportexport:RefreshButtonLabel'))

        HELPBUTTONLABEL = string(message('robotics:ikdesigner:dataimportexport:HelpButtonLabel'))

        CANCELBUTTONLABEL = string(message('robotics:ikdesigner:dataimportexport:CancelButtonLabel'))

        AVAILABLEWSVARSLABEL = string(message('robotics:ikdesigner:dataimportexport:AvailableWSVariablesLabel'))
    end

    methods
        function obj = CustomDataUI(appWindow, parentComponent)
        %CustomDataUI Constructor

            obj.AppWindow = appWindow;
            obj.Parent = parentComponent;
            obj.IsWindowOpen = false;

            obj.initialize;
        end

        function initialize(obj)
        %initialize Initialize the view

            obj.TableVariableSizes = [];
            obj.TableVariableClasses = [];
            obj.TableVariables = [];
            obj.IsWindowOpen = false;

        end

        function sendNotification(obj, eventName, eventData)
        %sendNotification Send notification
        %   This method tells the parent to send the notification. This
        %   ensures that the notifications are only ever sent by the
        %   main toolstrip object.

            obj.Parent.sendNotification(eventName, eventData);
        end

        function selectedTableVarNames = get.SelectedTableData(obj)
        %get.SelectedTableData Get method for SelectedTableData

            selectedTableVarNames = obj.TableVariables(obj.SelectedTableIndices);
        end
    end

    methods (Access = ?matlab.unittest.TestCase)
        function cancelButtonFcn(obj,~)
        %cancelButtonFcn Callback when the cancel button is selected
        %   This method sets the imported data properties to empty and
        %   closes the associated figure with a cancellation flag.

            closeFigure(obj, true);
        end

        function okButtonFcn(obj,~)
        %okButtonFcn Callback when the OK button is selected
        %   This method assigns the imported data properties and closes
        %   the associated figure with a non-cancellation flag.

            closeFigure(obj, false);
        end

        function selectTableRow(obj,evt)
        %selectTableRow Row selection callback for the workspace WSTable
        %   This method updates the properties of the object when a row
        %   is selected. The method also allows multiple rows to be
        %   selected so that more than one object may be imported at
        %   once.

            if ~isempty(evt.Source.Selection)
                obj.SelectedTableIndices = evt.Source.Selection;
                obj.ValidDataSelection = true;
            else
                obj.ValidDataSelection = false;
            end
            refreshOKButtonEnableState(obj);
        end
    end

    methods (Abstract, Access = protected)
        %actOnOKButtonSelection Function that indicates attempted action when OK button is clicked
        actOnOKButtonSelection
    end

    methods (Access = protected)

        function figGrid = createBaselineUI(obj)
        %createBaselineUI Create a UIFigure for loading known and custom data
        %   Create a UI figure that can be used to load data from the
        %   workspace or from a list of known values.

            obj.TableVariableSizes = [];
            obj.TableVariableClasses = [];
            obj.TableVariables = [];
            obj.SelectedTableIndices = [];

            obj.UIFigure = uifigure('Name', obj.WindowName, 'Scrollable',true);
            obj.UIFigure.WindowStyle = "modal";
            obj.UIFigure.Resize = matlab.lang.OnOffSwitchState.on;

            % Create a grid within which to place the objects in the figure
            figGrid = uigridlayout(obj.UIFigure);
            figGrid.ColumnWidth = {100, '1x', 100, 100};
            figGrid.RowHeight = {};

            % Initialize the figure position to include the grid padding
            obj.UIFigure.Position(4) = figGrid.Padding(2) + figGrid.Padding(4);
        end

        function buildDescriptionArea(obj, figGrid)
        %buildDescriptionArea Add a description to the uifigure
        %   Add a uilabel containing a description of the uifigure to
        %   instruct the user how to use the UI.

        % Add a row to the grid
            rowNum = numel(figGrid.RowHeight) + 1;
            figGrid.RowHeight{rowNum} = 'fit';

            %Add a text area in the first row of the grid
            descriptionText = uilabel(figGrid);
            descriptionText.Layout.Row = rowNum;
            descriptionText.Layout.Column = [1 4];
            descriptionText.Text = obj.DescriptionText;
            descriptionText.WordWrap = 'on';

            % Update the figure height. The uilabel updates too slowly to
            % use the sole input, so instead buffer it with the number of
            % carriage returns in the description text.
            expTextHeight = descriptionText.Position(4) + (count(obj.DescriptionText, newline)+1)*10;
            obj.UIFigure.Position(4) = obj.UIFigure.Position(4) + figGrid.RowSpacing + expTextHeight;
        end

        function buildVariableSelectionPanel(obj, figGrid, hasRefresh)
        %buildVariableSelectionPanel Add a variables-in-workspace table to the UI Figure
        %   This method adds a table for displaying workspace variables
        %   to the UI figure. The table is part of a panel instructing
        %   the user to select the appropriate variables for import.

        % Filter the variables so the right ones are shown
            obj.filterVariables;

            % Add a row to the grid
            rowNum = numel(figGrid.RowHeight) + 1;
            figGrid.RowHeight{rowNum} = 'fit';

            %Add a panel in the second to last row of the grid
            dataPanel = uipanel(figGrid,'Title',obj.TablePanelText);
            dataPanel.Layout.Row = rowNum;
            dataPanel.Layout.Column = [1 4];

            %Add a grid inside the panel
            buttonHeight = 24;
            wsPanelHeight = 200;
            dataPanelGrid = uigridlayout(dataPanel);
            dataPanelGrid.ColumnWidth = {100, '1x'};
            dataPanelGrid.RowHeight = {wsPanelHeight buttonHeight};

            % Add a table to the panel
            obj.VariableTable = uitable(...
                'Parent', dataPanelGrid,...
                'FontSize', 12,...
                'Enable','on',...
                'ColumnName',[obj.VARCOLLABEL, obj.SIZECOLLABEL, obj.CLASSCOLLABEL],...
                'ColumnFormat',{'char','char','char'},...
                'RowName',{},...
                'Visible','on',...
                'SelectionType','row',...
                'RowStriping','off',...
                'CellSelectionCallback',@(src,evt) selectTableRow(obj,evt),...
                'Data',[obj.TableVariables',obj.TableVariableSizes',obj.TableVariableClasses']);
            obj.VariableTable.Layout.Row = 1;
            obj.VariableTable.Layout.Column = [1 2];

            if hasRefresh
                % Add Refresh button
                obj.TableRefreshButton = uibutton(dataPanelGrid,'Text',obj.REFRESHBUTTONLABEL);
                obj.TableRefreshButton.ButtonPushedFcn = @(~,~)obj.refreshTableData();
                obj.TableRefreshButton.Layout.Row = 2;
                obj.TableRefreshButton.Layout.Column = 1;
            else
                buttonHeight = 0;
                dataPanelGrid.RowHeight{2} = 0;
            end

            % Update the figure height and the grid
            wsComponentHeight = 2*figGrid.RowSpacing + 2*dataPanelGrid.RowSpacing + dataPanel.Position(4) + buttonHeight;
            obj.UIFigure.Position(4) = obj.UIFigure.Position(4) + wsComponentHeight;

            % Since nothing is selected at the start, set the default
            % selection validity to false
            obj.ValidDataSelection = false;
        end

        function buildAcceptanceButtons(obj, figGrid)
        %buildAcceptanceButtons Add buttons to the uifigure
        %   This method adds the Help, OK, and Cancel buttons to the
        %   uifigure.

        % Add a row to the grid
            rowNum = numel(figGrid.RowHeight) + 1;
            figGrid.RowHeight{rowNum} = 'fit';

            % Add Cancel button
            obj.CancelButton = uibutton(figGrid,'Text',obj.CANCELBUTTONLABEL);
            obj.CancelButton.ButtonPushedFcn = @(~,evt)obj.cancelButtonFcn(evt);
            obj.CancelButton.Layout.Row = rowNum;
            obj.CancelButton.Layout.Column = 4;

            % Update the close request function, which is the X in the
            % upper right corner has the same utility as the cancel button
            obj.UIFigure.CloseRequestFcn = obj.CancelButton.ButtonPushedFcn;

            % Add the OK button
            obj.OKButton = uibutton(figGrid,'Text',obj.OKButtonText);
            obj.OKButton.ButtonPushedFcn = @(~,evt)obj.okButtonFcn;
            obj.OKButton.Layout.Row = rowNum;
            obj.OKButton.Layout.Column = 3;
            obj.OKButton.Enable = 'off';

            % Add the Help button
            helpButton = uibutton(figGrid,'Text',obj.HELPBUTTONLABEL);
            helpButton.ButtonPushedFcn = @(~,evt)helpview(fullfile(docroot,'robotics','helptargets.map'),obj.HelpDocID);
            helpButton.Layout.Row = rowNum;
            helpButton.Layout.Column = 1;
            helpButton.Enable = 'on';

            % Update the figure height
            obj.UIFigure.Position(4) = obj.UIFigure.Position(4) + figGrid.RowSpacing + obj.OKButton.Position(4);
        end

        function refreshOKButtonEnableState(obj)
        %refreshOKButtonEnableState Refresh button enable state
        %   This method checks whether the buttons should be enabled or
        %   disabled given the current data selection.

            if obj.ValidDataSelection
                obj.OKButton.Enable = 'on';
            else
                obj.OKButton.Enable = 'off';
            end
        end

        function filterVariables(obj) %#ok<MANU>
        %filterVariables Filter the variables so only those in the specified classes are listed
        %   By default, this is a no-op, but it may be overwritten in
        %   child classes.

        end
    end

    methods (Access = private)
        function closeFigure(obj, userCancelled)
        %closeWSFigure Close the workspace UI figure
        %   This method assigns the imported data properties using the
        %   selected variable names and notifies the parent toolstrip
        %   that the data has been imported. It also sets the user
        %   cancellation flag, closes the import dialog, and resets the
        %   app state.

        % Hide the app figure to send a visual indicator that the
        % action is done. However, the figure is not yet deleted
        % because some downstream actions may rely on its children.
            bringToFront(obj.AppWindow);
            obj.UIFigure.Visible= 'off';

            % Update properties
            if ~userCancelled
                % Try to export the data to the workspace and throw an
                % error if there are issues
                try
                    obj.actOnOKButtonSelection();
                catch ME
                    messageEvt = robotics.ikdesigner.internal.event.MessageEvent(ME.message);
                    obj.sendNotification("RequestUIAlert", messageEvt);
                end
            else
                obj.sendNotification("RequestAppNotBusy",[]);
            end

            % Close the figure and reset the app state
            delete(obj.UIFigure);

            % Allow new subsequent modal UIs
            obj.IsWindowOpen = false;
        end

        function refreshTableData(obj)
        %refreshTableData Callback for the optional refresh button

            obj.filterVariables();
            obj.VariableTable.Data = [obj.TableVariables',obj.TableVariableSizes',obj.TableVariableClasses'];
        end
    end
end
