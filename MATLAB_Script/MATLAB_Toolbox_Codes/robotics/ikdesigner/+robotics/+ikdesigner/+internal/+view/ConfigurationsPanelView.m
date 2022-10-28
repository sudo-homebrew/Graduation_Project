classdef ConfigurationsPanelView < robotics.ikdesigner.internal.view.View
%This function is for internal use only. It may be removed in the future.

%   ConfigurationsPanelView

%   Copyright 2021-2022 The MathWorks, Inc.

    events
        RequestStoreCurrentConfig

        RequestSnapToConfig

        DeleteConfig

        MoveConfig

        TableEdited

        RequestUIAlert
    end

    properties (GetAccess = ?matlab.unittest.TestCase, SetAccess = private)

        %Parent Parent UIFigure object handle
        Parent

        %KeyArray
        KeyArray
    end

    % View UI components
    properties (GetAccess = ?matlab.unittest.TestCase, SetAccess = private)

        %Grid Handle to uigridlayout used to organize the view
        Grid

        %Slider Handle to uislider used to navigate stored configurations
        Slider

        %SliderValueField Handle to uifield used to indicate current configuration index
        SliderValueField

        %SliderValueField Handle to uilabel used to indicate maximum configuration index
        SliderLabel

        %LeftButton Button to move slider back an index
        LeftButton

        %RightButton Button to move slider forward an index
        RightButton

        %ConfigurationsDropDown Dropdown of available configurations to store
        ConfigurationsDropDown

        %StoreConfigButton Button to store the configuration specified by the dropdown
        StoreConfigButton

        %SnapToConfigButton Button to snap to the selected configuration
        SnapToConfigButton

        %RemoveButton Button to remove to the selected configuration
        RemoveButton

        %UpButton Button to move configurations up in the table
        UpButton

        %DownButton Button to move configurations down in the table
        DownButton

        %ConfigurationsTable Table of configurations
        ConfigurationsTable

        %ConfigurationView
        ConfigurationView
    end

    properties (Dependent)

        %NumRows Number of rows in the configurations table
        NumRows

        %Enabled State indicating whether or not child ui components are enabled
        Enabled
    end

    properties (Access = private, Constant)

        DELETEICON = fullfile(robotics.ikdesigner.internal.constants.Data.ICONSDIRECTORY,'Delete.png')
        UPICON = fullfile(robotics.ikdesigner.internal.constants.Data.ICONSDIRECTORY,'Up_16.png')
        DOWNICON = fullfile(robotics.ikdesigner.internal.constants.Data.ICONSDIRECTORY,'Down_16.png')
        LEFTICON = fullfile(robotics.ikdesigner.internal.constants.Data.ICONSDIRECTORY,'Back_16.png')
        RIGHTICON = fullfile(robotics.ikdesigner.internal.constants.Data.ICONSDIRECTORY,'Forward_16.png')

        %Errors
        INVALIDCONFIGERR = message("robotics:ikdesigner:configurationspanel:InvalidConfig")
        INVALIDCONFIGNAMEERR = message("robotics:ikdesigner:configurationspanel:InvalidName")
        SELECTSINGLEROWERR = string(message("robotics:ikdesigner:configurationspanel:SelectSingleRow"))

        %Table column headers
        CONFIGCOLLABEL = string(message('robotics:ikdesigner:configurationspanel:ConfigColumnLabel'))
        COLLISIONCOLLABEL = string(message('robotics:ikdesigner:configurationspanel:CollisionStatusColumnLabel'))
        VALUECOLLABEL = string(message('robotics:ikdesigner:configurationspanel:ValueColumnLabel'))

        %Button labels
        STORECONFIGLABEL = string(message('robotics:ikdesigner:configurationspanel:StoreConfigLabel'))
        SNAPTOCONFIGLABEL = string(message('robotics:ikdesigner:configurationspanel:SnapToConfigLabel'))
        COLLISIONPASSLABEL = string(message('robotics:ikdesigner:configurationspanel:CollisionPassLabel'))
        COLLISIONFAILLABEL = string(message('robotics:ikdesigner:configurationspanel:CollisionFailLabel'))
        COLLISIONNOTRUNLABEL = string(message('robotics:ikdesigner:configurationspanel:CollisionNotRunLabel'))

        %Button tooltips
        STORECONFIGTOOLTIP = string(message('robotics:ikdesigner:configurationspanel:StoreConfigTooltip'))
        SNAPTOCONFIGTOOLTIP = string(message('robotics:ikdesigner:configurationspanel:SnapToConfigTooltip'))
        DELETECONFIGTOOLTIP = string(message('robotics:ikdesigner:configurationspanel:TableDeleteTooltip'))
        MOVEUPTOOLTIP = string(message('robotics:ikdesigner:configurationspanel:TableMoveUpTooltip'))
        MOVEDOWNTOOLTIP = string(message('robotics:ikdesigner:configurationspanel:TableMoveDownTooltip'))
        STEPFWDTOOLTIP = string(message('robotics:ikdesigner:configurationspanel:StepForwardTooltip'))
        STEPBACKTOOLTIP = string(message('robotics:ikdesigner:configurationspanel:StepBackwardTooltip'))
    end

    % Core view methods
    methods
        function obj = ConfigurationsPanelView(fig)
        %ConfigurationsPanelView Constructor

        % Create a grid and initialize a browser view
            obj.Parent = fig;

            obj.createViewComponents();

            % Disable all the buttons
            obj.Enabled = 'off';
        end

        function setup(obj, jointConfigAngularIdx, modelConfigsMap, configurationsKeyArray)
        %setup Reset data members and assign new values from incoming data

            obj.ConfigurationView = robotics.ikdesigner.internal.view.ConfigurationView(jointConfigAngularIdx);

            % The table data has to be initialized with the right data
            % type, otherwise it will take on a new data type despite the
            % column type specification
            obj.ConfigurationsTable.Data = string.empty;
            obj.KeyArray = string.empty;

            % Add model data to the table. The order is provided by the
            % order of the key array, so it will match the stored order
            obj.addConfigDataToTable(modelConfigsMap, configurationsKeyArray);
        end

        function initialize(obj)
        %initialize Initialize a new session

        % Enable all the buttons
            obj.Enabled = 'on';

        end

        function numRows = get.NumRows(obj)
        %get.NumRows Returns the number of rows in the table

            numRows = size(obj.ConfigurationsTable.Data,1);
        end

        function set.Enabled(obj, isEnabled)
        %set.Enabled Enable or disable the view's child UI components

            children = obj.Grid.Children;
            for i = 1:numel(children)
                childObj = children(i);
                if isa(childObj, 'matlab.ui.control.Slider')
                    % The slider doesn't have an "Enabled" property, so
                    % visibility is used instead. This hides the slider
                    % during the initialization phase.
                    childObj.Visible = isEnabled;
                else
                    childObj.Enable = isEnabled;
                end
            end
        end
    end

    % Public view methods used by client
    methods
        function updateTableValues(obj, eventData)
        %updateTableValues Modify the contents of the displayed configurations table
        %   This method is used by clients to add or edit values in the
        %   table. The method accepts an event that indicates the type
        %   of modification, then routes that event data to the proper
        %   internal method. The event type is an enumeration.

        % Get data from event
            updateType = eventData.UpdateType;
            modelConfigsMap = eventData.Map;
            keys = eventData.AffectedKeys;

            % Update the map
            switch updateType
              case robotics.ikdesigner.internal.model.ConfigUpdate.ValuesAdded
                obj.addConfigDataToTable(modelConfigsMap, keys);
              case robotics.ikdesigner.internal.model.ConfigUpdate.MapValuesChange
                obj.editTableValues(modelConfigsMap, keys);
            end
        end

        function updateRowData(obj, evt)
        %updateRowData Update rows in the configurations table
        %   This method is called when the user edits data in the
        %   configurations table. The corresponding event indicates not
        %   only the updated value, but the specific indices that have
        %   been edited. This method validates the inputs and notifies
        %   the model of the changes.

        % Get the event data
            row = evt.Indices(1);
            col = evt.Indices(2);

            % Take one set of actions if the edited column corresponds to
            % the column associated with configuration names
            if col == robotics.ikdesigner.internal.model.ConfigTableColAssignment.Name
                % Use a try/catch syntax to rethrow all errors as app
                % window alerts
                try
                    name = obj.validateName(obj.ConfigurationsTable.Data(row,1));
                catch ex
                    obj.ConfigurationsTable.Data(row,1) = evt.PreviousData;
                    obj.notifyUIAlert(ex.message);
                    return;
                end
            else
                name = obj.ConfigurationsTable.Data(row,1);
            end



            if col == robotics.ikdesigner.internal.model.ConfigTableColAssignment.Value
                % Use a try/catch syntax to rethrow all errors as app
                % window alerts
                try
                    prevNumCols = numel(str2num(evt.PreviousData)); %#ok<ST2NM>
                    config = obj.validateConfigString(obj.ConfigurationsTable.Data(row,3), prevNumCols);
                catch ex
                    obj.ConfigurationsTable.Data(row,3) = evt.PreviousData;
                    obj.notifyUIAlert(ex.message);
                    return;
                end
            else
                config = num2str(obj.ConfigurationsTable.Data(row,3));
            end

            key = obj.KeyArray(row);
            obj.notifyTableEdited(col, key, name, config);
        end

        function updateTableSelection(obj, evt)
        %updateTableSelection Method that is called when the cell selection is updated
        %   This method propagates changes in the table selection.
        %   Specifically, it moves the slider to reflect the currently
        %   selected row in the table.

            selectedRows = unique(evt.Indices(:,1));
            if numel(selectedRows) == 1
                obj.updateSliderPosition(selectedRows);
            end
        end
    end

    methods (Access = private)

        function addConfigDataToTable(obj, modelConfigsMap, keysToAdd)
        %addConfigDataToTable Update user-facing table to reflect additions made in the model
        %   This method is called when data is added to the model and
        %   the view is not yet up to date. This method adds
        %   configurations to the view to sync with the model. The
        %   configurations are appended to the bottom, as this is the
        %   only way new configurations can be added (they can be moved
        %   after being added).

            for i = 1:numel(keysToAdd)
                rowData = obj.getRowFromModelData(modelConfigsMap(keysToAdd(i)));

                % Append the value to the last row of the table
                newRowIdx = obj.appendRowToTable(keysToAdd(i), rowData);
            end

            % Update the slider display
            obj.updateSliderLimits();

            % Select the last value
            if obj.NumRows > 0
                obj.selectRow(newRowIdx);
            end

        end

        function editTableValues(obj, modelConfigsMap, keysToEdit)
        %editTableValues Update user-facing table to reflect edits made in the model
        %   Given the model configurations map and a set of keys that
        %   have been changed, this method iterates over those keys and
        %   updates the corresponding table values so that they
        %   correctly reflect the data in the model.

            for i = 1:numel(keysToEdit)
                keyToEdit = keysToEdit(i);
                rowData = obj.getRowFromModelData(modelConfigsMap(keyToEdit));
                rowToEditIdx = strcmp(keyToEdit, obj.KeyArray);
                obj.ConfigurationsTable.Data(rowToEditIdx,:) = rowData;
            end

        end

        function newRowIdx = appendRowToTable(obj, rowConfigKey, rowData)
        %appendRowToTable Add the specified row to the table
        %   This method adds the row data to the end of the table and
        %   adds the associated key to the key array. In the view, the
        %   key array is predominantly used to look up configuration
        %   keys given only the associated row data.

            newRowIdx = obj.NumRows+1;
            obj.ConfigurationsTable.Data(newRowIdx,:) = rowData;
            obj.KeyArray(newRowIdx) = rowConfigKey;
        end

        function gotoRow(obj, relPos)
        %gotoRow Select a row a distance RELPOS from the current selected row and snap to that configuration
        %   This method selects the row, given by its relative position
        %   to the currently selected row. The method also notifies the
        %   controller to snap to this newly selected row and update
        %   the associated visual states.

            % Make sure a row is selected
            selectedRowIdx = obj.validateSingleRowSelection();
            if isempty(selectedRowIdx)
                return;
            end
            rowToGoTo = selectedRowIdx + relPos;
            newRowIdx = obj.validateRowInTable(rowToGoTo);

            obj.snapToRowByIdx(newRowIdx)
        end

        function snapToRowByIdx(obj, rowIdx)

            % Notify the controller
            selectionKey = obj.KeyArray(rowIdx);
            obj.notifySnapToConfig(selectionKey);

            % Highlight the row
            obj.selectRow(rowIdx);
        end

        function updateSliderAfterMove(obj, src)

            newSliderPosition = src.Value;
            newRowIdx = round(newSliderPosition);

            % If the new position is within the range of available rows,
            % snap to it.
            if newRowIdx <= obj.NumRows
                obj.snapToRowByIdx(newRowIdx);
            end
            
        end

        function updateSliderDuringMove(obj, ~, evt)

            obj.SliderValueField.Value = min(round(evt.Value), obj.NumRows);

        end

        function deleteRow(obj)
        %deleteRow Delete the selected row from the configurations table

        % Make sure a row is selected
            selectedRowIdx = obj.validateSingleRowSelection();
            if isempty(selectedRowIdx)
                return;
            end
            selectionKey = obj.KeyArray(selectedRowIdx);

            % Remove the row from the user-facing table and key table
            obj.ConfigurationsTable.Data(selectedRowIdx,:) = [];
            obj.KeyArray(selectedRowIdx) = [];

            % Update the slider display
            obj.updateSliderLimits();

            % Update the selected field display
            selectedRowIdx = obj.validateSingleRowSelection();
            obj.updateSliderPosition(selectedRowIdx);

            % Update model
            obj.notifyDeleteConfig(selectionKey, selectedRowIdx);
        end

        function moveRow(obj, posChange)
        %moveRow Move the selected row by the number of rows specified
        %   Move the selected row by the number of rows given in
        %   posChange. This action occurs in the view, but the change
        %   is then sent to the model so it may update accordingly.

        % Make sure a row is selected
            selectedRowIdx = obj.validateSingleRowSelection();
            if isempty(selectedRowIdx)
                return;
            end
            selectionKey = obj.KeyArray(selectedRowIdx);

            % Get the new row position and cap it to the allowable limits
            newRowIdx = selectedRowIdx + posChange;
            newRowIdx = obj.validateRowInTable(newRowIdx);

            % Remove the selected index from the indices of all the rows
            % and move it to the new position. Then reassign the table
            tableRowIndices = 1:obj.NumRows;
            tableRowIndices(selectedRowIdx) = [];
            rowIndicesForNewTable = [tableRowIndices(1:(newRowIdx-1)) selectedRowIdx tableRowIndices(newRowIdx:end)];
            obj.ConfigurationsTable.Data = obj.ConfigurationsTable.Data(rowIndicesForNewTable, :);
            obj.KeyArray = obj.KeyArray(rowIndicesForNewTable);

            % Select the new row
            obj.selectRow(newRowIdx);

            % Update model
            obj.notifyMoveConfig(selectionKey, rowIndicesForNewTable);
        end
    end

    methods (Access = private)
        function notifyStoreConfig(obj)
        %notifyStoreConfig Notify controller to store the robot's present joint configuration

            notify(obj, 'RequestStoreCurrentConfig');
        end

        function notifySnapToConfig(obj, configKey)
        %notifySnapToConfig Notify controller to snap to the configuration selected in the table

            data = robotics.ikdesigner.internal.event.KeyBasedEventData(configKey);
            notify(obj, 'RequestSnapToConfig', data);
        end

        function notifyDeleteConfig(obj, selectedKey, newRowIndices)
        %notifyDeleteConfig Tell the controller to delete the stored configuration from the model
        %   In the current app, each configuration is used exactly
        %   once, so any time a row is deleted, a configuration is also
        %   deleted. If this were to change, the value of the selected
        %   key could be filtered here.

            data = robotics.ikdesigner.internal.event.ConfigTableEventData(selectedKey, newRowIndices);
            notify(obj, 'DeleteConfig', data);
        end

        function notifyMoveConfig(obj, keys, newRowIndices)
        %notifyMoveConfig Tell the model to update its order array

            data = robotics.ikdesigner.internal.event.ConfigTableEventData(keys, newRowIndices);
            notify(obj, 'MoveConfig', data);
        end

        function notifyTableEdited(obj, columnEdited, selectedKey, newName, newConfig)
        %notifyTableEdited Notify controller that configurations table has been edited

            data = robotics.ikdesigner.internal.event.ConfigTableEventData(selectedKey, []);
            data.NewName = newName;
            data.NewConfig = newConfig;
            data.ValueChanged = columnEdited;
            notify(obj, 'TableEdited', data);

        end

        function notifyUIAlert(obj, messageString)
        %notifyUIAlert Notify controller that the app window needs to issue a UI alert

            messageEvt = robotics.ikdesigner.internal.event.MessageEvent(messageString);
            notify(obj, 'RequestUIAlert', messageEvt);
        end
    end

    methods (Access = private)
        function selectRow(obj, rowToSelect)
        %selectRow Manually specify selection

            ncols = size(obj.ConfigurationsTable.Data,2);
            obj.ConfigurationsTable.Selection = [repmat(rowToSelect, ncols, 1) (1:ncols)'];

            obj.updateSliderPosition(rowToSelect);
        end

        function row = validateSingleRowSelection(obj)
        %validateSingleRowSelection Verify that a single row is selected and return that row
        %   If no rows are selected, the method returns empty. If
        %   multiple rows are selected, this method throws an error
        %   indicating to the user that they should choose just one
        %   row.

            if isempty(obj.ConfigurationsTable.Selection)
                row = [];
                return;
            end

            selectedRows = obj.ConfigurationsTable.Selection(:,1);
            if numel(unique(selectedRows)) > 1
                row = [];
                obj.notifyUIAlert(obj.SELECTSINGLEROWERR);
            else
                % Return the first value since they're all the same
                row = selectedRows(1);
            end
        end

        function newRowIdx = validateRowInTable(obj, newRowIdx)
        %validateRowInTable Verify that a new row is in the table return that row
        %   Given a row index, this method bounds it given the overall
        %   table size.

            newRowIdx = max(1, newRowIdx);
            newRowIdx = min(obj.NumRows, newRowIdx);
        end

        function updateSliderLimits(obj)
        %updateSliderLimits Update displayed and text slider limits

        % Update the slider limit displayed in the label
            obj.SliderLabel.Text = sprintf('/ %i', obj.NumRows);

            % The displayed limits must have a non-zero range
            displayMaxLim = max(obj.NumRows, 2);
            obj.Slider.Limits = [1 displayMaxLim];
        end

        function updateSliderPosition(obj, rowIdx)
        %updateSliderPosition Move the slider to reflect the selected row

            if isempty(rowIdx)
                % Nothing is selected
                obj.Slider.Value = obj.Slider.Limits(1);
                obj.SliderValueField.Value = 0;
            else
                obj.Slider.Value = rowIdx;
                obj.SliderValueField.Value = rowIdx;
            end
        end

        function createViewComponents(obj)
        %createViewComponents Add the visual user-facing elements
        %   This panel contains a figure with a slider, buttons, and a
        %   table. The items in the figure are arranged using a UI
        %   grid, which ensures that items scale cleanly with resizing.

            import robotics.ikdesigner.internal.helpers.*

            obj.Grid = uigridlayout(obj.Parent, 'ColumnWidth', {30, 'fit', 'fit', '1x', 30, 30, 30, 40}, 'Rowheight', {'fit', 20, '1x', 25, 25});

            % Add the slider to the first row of the grid
            obj.addSlider(obj.Grid, 1);

            % Add the Store Configuration button
            obj.StoreConfigButton = uibutton(obj.Grid, 'push', ...
                                             'Text', obj.STORECONFIGLABEL, ...
                                             'Position', [0 0 150 30]);
            obj.StoreConfigButton.Tooltip = obj.STORECONFIGTOOLTIP;
            obj.StoreConfigButton.Layout.Row = 2;
            obj.StoreConfigButton.Layout.Column = [1 2];
            obj.StoreConfigButton.ButtonPushedFcn = @(src,evt)obj.notifyStoreConfig();

            % Add Snap to Configuration button
            obj.SnapToConfigButton = uibutton(obj.Grid, 'push', ...
                                              'Text', obj.SNAPTOCONFIGLABEL, ...
                                              'Position', [0 0 150 30]);
            obj.SnapToConfigButton.Tooltip = obj.SNAPTOCONFIGTOOLTIP;
            obj.SnapToConfigButton.Layout.Row = 2;
            obj.SnapToConfigButton.Layout.Column = 3;
            obj.SnapToConfigButton.ButtonPushedFcn = @(src,evt)obj.gotoRow(0);

            % Add Remove button
            obj.RemoveButton = uibutton(obj.Grid, 'push', 'Text', '', 'Icon', obj.DELETEICON);
            obj.RemoveButton.Tooltip = obj.DELETECONFIGTOOLTIP;
            obj.RemoveButton.Layout.Row = 2;
            obj.RemoveButton.Layout.Column = 7;
            obj.RemoveButton.ButtonPushedFcn = @(src,evt)obj.deleteRow();

            % Add the table
            obj.ConfigurationsTable = uitable(obj.Grid);
            obj.ConfigurationsTable.Layout.Row = [3 5];
            obj.ConfigurationsTable.Layout.Column = [1 7];
            obj.ConfigurationsTable.ColumnEditable=[true, false, true];
            obj.ConfigurationsTable.ColumnName = [obj.CONFIGCOLLABEL; obj.COLLISIONCOLLABEL; obj.VALUECOLLABEL];
            obj.ConfigurationsTable.ColumnFormat = {'char' 'char' 'char'};
            obj.ConfigurationsTable.ColumnWidth = {'fit','fit','1x'};
            obj.ConfigurationsTable.CellSelectionCallback = @(src,evt)obj.updateTableSelection(evt);
            obj.ConfigurationsTable.CellEditCallback = @(src,evt)obj.updateRowData(evt);

            % Add the Up button
            obj.UpButton = uibutton(obj.Grid, 'push', 'Text', '', 'Icon', obj.UPICON);
            obj.UpButton.Tooltip = obj.MOVEUPTOOLTIP;
            obj.UpButton.Layout.Row = 4;
            obj.UpButton.Layout.Column = 8;
            obj.UpButton.ButtonPushedFcn = @(src,event)obj.moveRow(-1);

            % Add the Down button
            obj.DownButton = uibutton(obj.Grid, 'push', 'Text', '', 'Icon', obj.DOWNICON);
            obj.DownButton.Tooltip = obj.MOVEDOWNTOOLTIP;
            obj.DownButton.Layout.Row = 5;
            obj.DownButton.Layout.Column = 8;
            obj.DownButton.ButtonPushedFcn = @(src,event)obj.moveRow(1);
        end

        function addSlider(obj, parent, row)
        %addSlider Add slider visual to the selected row

            import robotics.ikdesigner.internal.helpers.*

            % Add the slider
            obj.Slider = uislider(parent, 'MajorTicks', [], 'MinorTicks', []);
            obj.Slider.Layout.Column = [2 4];
            obj.Slider.Layout.Row = row;
            obj.Slider.Limits = [1 1.01];
            obj.Slider.ValueChangingFcn = @(src,evt)obj.updateSliderDuringMove(src,evt);
            obj.Slider.ValueChangedFcn = @(src,evt)obj.updateSliderAfterMove(src);

            % Add the Back button
            backButton = uibutton(obj.Grid, 'push', 'Text', '', 'Icon', obj.LEFTICON);
            backButton.Tooltip = obj.STEPBACKTOOLTIP;
            backButton.Layout.Row = row;
            backButton.Layout.Column = 1;
            backButton.ButtonPushedFcn = @(btn,event)obj.gotoRow(-1);

            % Add the Forward button
            forwardButton = uibutton(obj.Grid, 'push', 'Text', '', 'Icon', obj.RIGHTICON);
            forwardButton.Tooltip = obj.STEPFWDTOOLTIP;
            forwardButton.Layout.Row = row;
            forwardButton.Layout.Column = 5;
            forwardButton.ButtonPushedFcn = @(btn,event)obj.gotoRow(1);

            % Add the slider value field
            obj.SliderValueField = uieditfield(parent, 'numeric', 'Value', 1);
            obj.SliderValueField.Layout.Column = [6 7];
            obj.SliderValueField.Layout.Row = row;

            % Add the slider label
            obj.SliderLabel = uilabel(parent, 'Text', '/ 0');
            obj.SliderLabel.Layout.Column = 8;
            obj.SliderLabel.Layout.Row = row;
        end

        function rowData = getRowFromModelData(obj, modelMapData)
        %getRowFromModelData Construct a table row given model data
        %   This method translates the data stored in each entry of the
        %   configurations map into a string array that can be used to
        %   update the table.

        % Get data from model
            configName = modelMapData.Name;
            configValue = obj.ConfigurationView.getUserFacingConfigString(modelMapData.Configuration);
            switch modelMapData.CollisionState
              case robotics.ikdesigner.internal.model.RBTCollisionState.CollisionFree
                collisionState = obj.COLLISIONPASSLABEL;
              case robotics.ikdesigner.internal.model.RBTCollisionState.InCollision
                collisionState = obj.COLLISIONFAILLABEL;
              case robotics.ikdesigner.internal.model.RBTCollisionState.NotEvaluated
                collisionState = obj.COLLISIONNOTRUNLABEL;
            end

            rowData = [configName  collisionState  configValue];

        end

        function numericConfig = validateConfigString(obj, configString, numCols)
        %validateConfigString Validate a string that represents a configuration
        %   The configurations must be provided as strings since the
        %   tables are otherwise unable to display vectors in expanded
        %   form. Therefore it is necessary to do some custom
        %   validation on the text form of the vector. Additionally, it
        %   is known that the vector must be the same dimension as the
        %   original vector since the tree is immutable.

            numericConfig = str2num(configString); %#ok<ST2NM>
            if isempty(numericConfig)
                error(obj.INVALIDCONFIGERR);
            end
            validateattributes(numericConfig, {'numeric'}, {'vector','ncols',numCols,'real','finite','nonnan'});
            numericConfig = obj.ConfigurationView.getInternalConfig(configString);
        end

        function nameString = validateName(obj, nameString)
        %validateName Validate the user-specified configuration name label

            isValidChar = isstrprop(nameString,'alphanum') | isstrprop(nameString,'wspace');
            if any(~isValidChar)
                error(obj.INVALIDCONFIGNAMEERR);
            end
        end
    end
end
