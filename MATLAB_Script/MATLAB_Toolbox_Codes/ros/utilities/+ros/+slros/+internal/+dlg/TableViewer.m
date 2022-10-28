classdef TableViewer < robotics.core.internal.mixin.SetProperties
%This class is for internal use only. It may be removed in the future.

%TableViewer Opens a DDG dialog that lets the user select a row from a table.
%
%  Sample use:
%   selector = ros.slros.internal.dlg.TableViewer(["Col1", "Col2"], "Title", "Select Something");
%   data = table(["String1"; "String2"], ["1"; "2"])
%   selector.updateData(data)
%   selector.openDialog(@(isAccepted,data) disp(data))

%   Copyright 2017-2018 The MathWorks, Inc.

    properties (Constant, Access = {?ros.slros.internal.dlg.TableViewer, ...
                            ?matlab.unittest.TestCase})
        % Set the size of the dialog to a reasonable value
        % Users can always manually resize the dialog as needed.

        %EnabledColor
        EnabledColor = [255, 255, 255]

        %DisabledColor
        DisabledColor = [230, 230, 230]

        %TableTag
        TableTag = 'datatable'
    end

    properties (Access = private)
        %Stretchable
        StretchableInternal (1,:) double = 0
    end

    properties (SetAccess = immutable)

        %Header
        Header string = ""

    end

    properties(Access = {?ros.slros.internal.dlg.TableViewer, ...
                         ?matlab.unittest.TestCase})
        %SelectedRow 1-based index of the currently selected row
        SelectedRow = 0
    end

    properties
        %Dialog - Handle to the DDG Dialog
        Dialog

        %CloseFcnHandle - Function that will be called when dialog closes
        %   The function has to follow this signature:
        %   closeFcn(isAcceptedSelection, selectedRowData)
        %
        %   selectedRowData will be [] if the user presses the cancel
        %   button.
        CloseFcnHandle (1,:) function_handle = function_handle.empty(1,0)

        %Title - Title of dialog
        Title (1,1) string = ""

        %DataTable - Actual data stored in table
        DataTable table

        %RowEnabled
        RowEnabled logical

        %HelpMethod - Help method that should be called
        HelpMethod (1,:) char

        %HelpId - Anchor ID that should be called
        HelpId (1,:) char

        %DialogSize - Width and height of dialog (in pixels)
        %   This is a 2-vector [width, height]
        DialogSize (1,2) double

        %DialogPosition
        DialogPosition

        %HeaderVisibility - Determine visibility of [row col] headers
        HeaderVisibility (1,2) double

        %LastColumnStretchable - Does last column fill the remaining width?
        %   Setting this to "true" can be helpful when you want to use the
        %   table more like a listbox.
        LastColumnStretchable logical

        %InitialRowSelection - Selected row on first opening of dialog
        %   This is a 1-based index of the table row. By default, the first
        %   row is selected.
        %   If this index does not exist when the dialog opens, no
        %   row is selected.
        InitialRowSelection double {mustBePositive, mustBeInteger}
    end

    properties (Dependent)
        %NumColumns
        NumColumns = 1

        %Stretchable
        Stretchable
    end

    properties (Access = protected)
        %ConstructorProperties - Allowable name-value pairs
        ConstructorProperties = {'Title', 'Stretchable', 'HelpMethod', ...
                            'HelpId', 'DialogPosition', 'DialogSize', ...
                            'HeaderVisibility', 'LastColumnStretchable', ...
                            'InitialRowSelection'};

        %ConstructorPropertyDefaultValues - Default value for name-value pairs
        ConstructorPropertyDefaultValues = {...
            '', ...                         % Title
            0, ...                          % Stretchable
            '', ...                         % HelpMethod
            '', ...                         % HelpId
            [], ...                         % DialogPosition
            [800 400], ...                  % DialogSize
            [0 1], ...                      % HeaderVisibility
            false, ...                      % LastColumnStretchable
            1, ...                          % InitialRowSelection
                   }
    end

    methods
        function obj = TableViewer(header, varargin)
        %TableViewer(header, Name, Value, ...)
            obj.Header = header;
            obj.StretchableInternal = zeros(1, obj.NumColumns);

            % Reset default for Stretchable property based on number of columns
            obj.ConstructorPropertyDefaultValues{2} = obj.StretchableInternal;

            obj.setProperties(numel(varargin), varargin{:});
            obj.updateData(array2table(repmat("", 1, obj.NumColumns)));
        end

        function val = get.NumColumns(obj)
            val = numel(obj.Header);
        end

        function set.Stretchable(obj, val)
            validateattributes(val, {'double'}, {'numel', obj.NumColumns}, ...
                               'TableViewer', 'Stretchable');
            obj.StretchableInternal = val;
        end

        function val = get.Stretchable(obj)
            val = obj.StretchableInternal;
        end

        function openDialog(obj, closeFcnHandle)
        % closeFcnHandle: handle to function that takes two arguments
        %   closeFcn(isAcceptedSelection, selectedRowData)
        %
        %   selectedRowData will be [] if the user presses the cancel
        %   button.
            if nargin == 2
                validateattributes(closeFcnHandle, {'function_handle'}, {'scalar'});
                obj.CloseFcnHandle = closeFcnHandle;
            end
            obj.Dialog = DAStudio.Dialog(obj);
            obj.refresh();

            % Select the table row as specified by user
            if obj.InitialRowSelection <= height(obj.DataTable)
                obj.Dialog.ensureTableRowVisible(obj.TableTag, obj.InitialRowSelection - 1)
                obj.Dialog.selectTableRow(obj.TableTag, obj.InitialRowSelection - 1);
            end
        end

        function updateData(obj, data, enabled)
        %updateData
            validateattributes(data, {'table'}, ...
                               {'ncols', obj.NumColumns}, ...
                               'updateData', 'data');
            if nargin == 3
                validateattributes(enabled, {'logical'}, ...
                                   {'numel', size(data,1)}, ...
                                   'updateData', 'enabled');
            else
                enabled = true(size(data,1),1);
            end
            obj.DataTable = data;
            obj.RowEnabled = enabled;
            obj.refresh();
        end

        function refresh(obj)
        %refresh Refresh the dialog if it exists
            if isa(obj.Dialog, 'DAStudio.Dialog')
                obj.Dialog.refresh()
                obj.Dialog.selectTableRow(obj.TableTag, obj.SelectedRow - 1);
            end
        end

        function tableSelectionChangedCallback(obj, dlg, tableTag)
        %tableSelectionChangedCallback Called for right-click or key presses
        %   Change the selection to emulate similar behavior in
        %   listbox.
            obj.SelectedRow = dlg.getSelectedTableRow(tableTag) + 1;
            obj.Dialog.selectTableRow(obj.TableTag, obj.SelectedRow - 1);
        end

        function tableCurrentItemChangedCallback(obj, ~, row, ~)
        %tableSelectionChangedCallback Called when user selects an item from the table
        %   This is triggered by a single left mouse click.
            obj.SelectedRow = row + 1;
            obj.Dialog.selectTableRow(obj.TableTag, obj.SelectedRow - 1);
        end

        function tableDoubleClickedCallback(obj, dlg, row, ~, ~)
        %tableDoubleClickedCallback Called when user double-clicks left mouse button

            if row == dlg.getSelectedTableRow(obj.TableTag)
                obj.dlgClose('ok');
            end
            obj.Dialog.delete();
        end

        function dlgstruct = getDialogSchema(obj)
        %getDialogSchema Construct the dialog
        %   It only consists of a single table showing parameter names
        %   and associated data types.

        % Main dialog
            dlgstruct.DialogTitle = obj.Title;
            % Buttons to show on dialog (these are options to pass to DDG,
            % not the final strings, so there is no need to use message
            % catalog)
            if ~(isempty(obj.HelpMethod) || isempty(obj.HelpId))
                dlgstruct.HelpMethod = obj.HelpMethod;
                dlgstruct.HelpArgs =  {obj.HelpId};  % doc topic id
                buttonset = {'Ok', 'Cancel', 'Help'};
            else
                buttonset = {'Ok', 'Cancel'};
            end
            dlgstruct.CloseMethod = 'dlgClose';
            dlgstruct.CloseMethodArgs = {'%closeaction'};
            dlgstruct.CloseMethodArgsDT = {'string'};

            % Make this dialog modal wrt to other DDG dialogs
            % (doesn't block MATLAB command line)
            dlgstruct.Sticky = true;

            % Construct the table widget
            mytable.Type  = 'table';
            mytable.Tag = obj.TableTag;

            dlgstruct.StandaloneButtonSet = buttonset;
            dlgstruct.MinMaxButtons = true;
            tabledata = cell(size(obj.DataTable,1), obj.NumColumns);
            tableColHeader = cellstr(obj.Header);
            tableColumnStretchable = obj.Stretchable;
            tableHeaderVisibility = obj.HeaderVisibility;
            for j = 1:size(obj.DataTable,1)
                if obj.RowEnabled(j)
                    backgroundColor = obj.EnabledColor;
                else
                    backgroundColor = obj.DisabledColor;
                end
                for k = 1:obj.NumColumns
                    % Topic Name
                    tabledata{j,k}.Type  = 'text';
                    tabledata{j,k}.Name = char(obj.DataTable{j,k});
                    tabledata{j,k}.Enabled  = obj.RowEnabled(j);
                    tabledata{j,k}.BackgroundColor = backgroundColor;
                end
            end

            mytable.Size  = size(tabledata);
            mytable.ColHeader = tableColHeader;
            mytable.Editable = true;
            mytable.Data = tabledata;
            mytable.SelectionChangedCallback = @obj.tableSelectionChangedCallback;
            mytable.CurrentItemChangedCallback = @obj.tableCurrentItemChangedCallback;
            mytable.ItemDoubleClickedCallback = @obj.tableDoubleClickedCallback;

            % Only allow the user to select rows
            mytable.SelectionBehavior = 'row';
            mytable.Grid = true;
            mytable.HeaderVisibility = tableHeaderVisibility;

            % Make parameter name column stretchable, but keep the type
            % column fixed (since all data type strings are fairly short
            mytable.ColumnStretchable = tableColumnStretchable;
            mytable.LastColumnStretchable = obj.LastColumnStretchable;

            mytable.SelectedRow = obj.InitialRowSelection - 1;

            % The dialog only contains the table
            dlgstruct.Items = {mytable};
            % Set size of dialog
            dlgstruct.Geometry = obj.getDialogGeometry;
        end

        function dlgClose(obj, closeaction)
        % closeaction is 'ok' if user clicked OK
        %                'cancel' if user clicked cancel or closed window

            if ~isempty(obj.CloseFcnHandle)
                isAcceptedSelection = strcmpi(closeaction, 'ok') && obj.SelectedRow ~= 0;
                try
                    if isAcceptedSelection
                        feval(obj.CloseFcnHandle, isAcceptedSelection, obj.DataTable(obj.SelectedRow,:));
                    else
                        feval(obj.CloseFcnHandle, isAcceptedSelection, []);
                    end
                catch ME
                    % Absorb all errors. If they are propagated back to
                    % DDG, this causes MATLAB to crash, (Can't convert to
                    % warnings as they are not displayed either).
                    disp(ME.message);
                end
            end
            % Since CloseFcnHandle was set in openDialog, we clean it up
            % when closing the dialog.
            obj.CloseFcnHandle = function_handle.empty();
        end
    end

    methods (Access = private)
        function geometry = getDialogGeometry(obj)
            dlgWidth = obj.DialogSize(1);
            dlgHeight = obj.DialogSize(2);
            if isempty(obj.DialogPosition)
                % Center the window on the screen
                ss = get(0, 'ScreenSize');
                xCenter = ss(3)/2;
                yCenter = ss(4)/2;
            else
                xCenter = obj.DialogPosition(1);
                yCenter = obj.DialogPosition(2);
            end


            x = xCenter - dlgWidth/2;
            y = yCenter - dlgHeight/2;

            geometry = [x, y, dlgWidth, dlgHeight];
        end
    end
end
