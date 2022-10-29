classdef DataImportView < robotics.ikdesigner.internal.view.CustomDataUI
%This function is for internal use only and may be removed in a future release

%DataImportView Views for handling data import from file and from the workspace
%   The Data Import View is a tool for creating a UI that displays a
%   list of objects in the workspace. The object has several properties
%   that customize its behavior: what objects are displayed, what event
%   is triggered upon completion, and what text is displayed. After
%   creating the object, define these properties, then call
%   createUIFromWorkspace, which creates the view. When completed, the
%   object will send a NewDataEvent with the workspace variables and
%   their names under the specified event moniker given in
%   DataImportEventName.

%   Copyright 2021-2022 The MathWorks, Inc.

    properties

        %DataTypeFilter Cell array of data types to be filtered
        %   When the Data Import View is loaded, it limits the scope of
        %   displayed data to just those that are of the type specified by
        %   the DataTypeFilter property. This property is listed as a cell
        %   array of possible datatypes.
        DataTypeFilter

        %DataImportEventName Event to be triggered once data has been selected
        DataImportEventName

        %DataValidationFcn Function used to validate imported data
        DataValidationFcn
    end


    properties (Access = {?matlab.unittest.TestCase, ?robotics_tests.app.ikdesigner.DataImportWindowTester})

        %WSImportFcn Function that is called to import data from workspace
        WSImportFcn

        %WSNamesFcn Function that is called to associated names to imported data
        WSNamesFcn

    end

    properties (Constant, Access = protected)
        WSVARSDROPDOWNLABEL = string(message('robotics:ikdesigner:dataimportexport:LoadFromWSDropdownLabel'))
    end

    methods
        function obj = DataImportView(appWindow, parentComponent)
        %DataImportView Constructor

            obj@robotics.ikdesigner.internal.view.CustomDataUI(appWindow, parentComponent);

            % Set default validation function
            obj.DataValidationFcn = @(x)(x);
        end
    end

    methods (Access = {?robotics.ikdesigner.internal.view.View, ?robotics.ikdesigner.internal.toolstrip.Tab, ?matlab.unittest.TestCase})

        function createUIFromWorkSpace(obj)
        %createUIFromWorkSpace Create UI for loading data from workspace

            if obj.IsWindowOpen
                return;
            end
            
            figGrid = obj.createBaselineUI();

            % Build all the components
            obj.buildDescriptionArea(figGrid);
            hasRefreshButton = true;
            obj.buildVariableSelectionPanel(figGrid, hasRefreshButton);
            obj.buildAcceptanceButtons(figGrid);

            % Initialize import callbacks
            obj.WSImportFcn = @()obj.loadSelectedWorkspaceData();
            obj.WSNamesFcn = @()(obj.SelectedTableData);
            obj.DataValidationFcn = @(x)(x);
        end

        function createUIFromList(obj, listNames, listNameCallbacks)
        %createUIFromWorkSpace Create UI for loading data from an existing list

            if obj.IsWindowOpen
                return;
            end
            
            figGrid = obj.createBaselineUI();

            % Build all the components
            obj.buildDescriptionArea(figGrid);
            obj.buildSelectionDropdown(figGrid, listNames, listNameCallbacks);
            hasRefreshButton = true;
            obj.buildVariableSelectionPanel(figGrid, hasRefreshButton);
            obj.buildAcceptanceButtons(figGrid);

            % Initialize import callbacks
            obj.WSImportFcn = @()obj.loadSelectedWorkspaceData();
            obj.WSNamesFcn = @()(obj.SelectedTableData);
            obj.DataValidationFcn = @(x)(x);
        end
    end

    methods (Access = protected)
        function actOnOKButtonSelection(obj)
            importedData = obj.WSImportFcn();
            importedDataNames = obj.WSNamesFcn();
            obj.DataValidationFcn(importedData);
            eventData = robotics.ikdesigner.internal.event.NewDataEvent(importedData, importedDataNames);
            obj.sendNotification(obj.DataImportEventName, eventData);
        end

        function filterVariables(obj)
        %filterVariables Filter the variables so only those in the specified classes are listed

            vars = evalin('base','whos');
            obj.TableVariables = {};
            obj.TableVariableSizes = {};
            obj.TableVariableClasses = {};

            % Iterate over all variables in the base workspace and check if
            % they fall into one of the acceptable classes
            for i = 1:numel(vars)
                for j = 1:numel(obj.DataTypeFilter)
                    isValidVariable = evalin('base', sprintf('isa(%s, ''%s'')', vars(i).name, obj.DataTypeFilter{j}));

                    % For each valid variable, store the name, size, and
                    % class in properties that will be used to update the
                    % table in the import dialog window
                    if isValidVariable
                        obj.TableVariables{end+1} = vars(i).name;
                        obj.TableVariableSizes{end+1} = num2str(vars(i).size);
                        obj.TableVariableClasses{end+1} = vars(i).class;
                        break;
                    end
                end
            end
        end

        function refreshOKButtonEnableState(obj)
        %refreshOKButtonEnableState Refresh button enable state
        %   This method checks whether the buttons should be enabled or
        %   disabled given the current data selection.

            refreshOKButtonEnableState@robotics.ikdesigner.internal.view.CustomDataUI(obj);

            if strcmp(obj.VariableTable.Enable, 'off')
                obj.OKButton.Enable = 'on';
            end
        end
    end

    methods (Access = private)
        function updateUIWorkspaceDisplay(obj, sourceEvt, loadFromWSItemData)
        %updateUIWorkspaceDisplay Dropdown callback to update UI properties dependent on the workspace table display
        %   Update the workspace table and button visibility properties
        %   and associated callbacks based on whether or not the
        %   workspace table should be active. This method is called
        %   from the dropdown, and has essentially two states: (1) when
        %   the "Load from workspace" view is active, the table should
        %   be editable and the associated data-loading methods are
        %   used for data input and naming; (2) when one of the custom
        %   dropdown values is selected, the table should be disabled,
        %   OK should be enabled, and the data import/naming functions
        %   are taken from the associated dropdown value (its items
        %   data).

            if isequal(sourceEvt.Value{1}, loadFromWSItemData)
                obj.VariableTable.Enable = 'on';
                obj.TableRefreshButton.Enable = 'on';
            else
                obj.VariableTable.Enable = 'off';
                obj.TableRefreshButton.Enable = 'off';
            end

            % Update the import function callbacks
            obj.WSImportFcn = sourceEvt.Value{1};
            obj.WSNamesFcn = sourceEvt.Value{2};
            refreshOKButtonEnableState(obj);
        end

        function importedData = loadSelectedWorkspaceData(obj)
        %loadSelectedWorkspaceData Load selected data from workspace
        %   Function to load get the values of data imported from the
        %   workspace. The method takes the cell array of selected
        %   variables in the workspace table and imports them, storing
        %   the values in a cell array.

            importedData = cellfun(@(x)evalin('base', x), obj.SelectedTableData, 'UniformOutput',false);
        end

        function buildSelectionDropdown(obj, figGrid, itemsArray, itemsDataArray)
        %buildSelectionDropdown Add a dropdown for user input options
        %   This method adds a uidropdown that allows the user to pick
        %   from existing data values or provide custom inputs in the
        %   workspace. The method takes the data values as input,
        %   accepting two arrays: a string array of item names, and a
        %   cell array of the corresponding items data. The format of
        %   the items data in this case is a cell array of 2-element
        %   cells, where the first value in each cell is the callback
        %   function that will load the corresponding object into
        %   MATLAB, and the second value is the name that will be
        %   associated with that data.

        % Add a row to the grid
            rowNum = numel(figGrid.RowHeight) + 1;
            figGrid.RowHeight{rowNum} = 'fit';

            % Add a dropdown in the second row of the grid
            robotDropdown = uidropdown(figGrid);
            robotDropdown.Layout.Row = rowNum;
            robotDropdown.Layout.Column = [1 4];

            % Populate the dropdown with items, including the option to
            % load from the workspace
            loadFromWSText = obj.WSVARSDROPDOWNLABEL;
            loadFromWSCB = @()obj.loadSelectedWorkspaceData;
            varNamesFromWSCB = @(obj)(obj.WSSelectedData);
            robotDropdown.Items = [loadFromWSText itemsArray];
            robotDropdown.ItemsData = [{{loadFromWSCB varNamesFromWSCB}} itemsDataArray];

            % Ensure the figure updates table accessibility based on
            % dropdown selection
            robotDropdown.ValueChangedFcn = @(evt,src)obj.updateUIWorkspaceDisplay(src, loadFromWSCB);

            % Update the figure height
            obj.UIFigure.Position(4) = obj.UIFigure.Position(4) + figGrid.RowSpacing + robotDropdown.Position(4);
        end
    end
end
