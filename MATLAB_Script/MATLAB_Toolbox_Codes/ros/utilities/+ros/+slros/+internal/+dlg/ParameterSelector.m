classdef ParameterSelector < handle
%This class is for internal use only. It may be removed in the future.

%ParameterSelector opens a DDG dialog that lets the user select from a
%  list of available ROS topics. Once the user accepts the changes (or
%  cancels the dialog), a callback is invoked with the closure action
%  and selected topic type.
%
%  Sample use:
%   selector = ros.slros.internal.dlg.ParameterSelector;
%   selector.openDialog(@(isAccepted,topicName) disp(topicName))

%   Copyright 2015-2018 The MathWorks, Inc.

    properties (Constant, Access = private)
        % Set the size of the dialog to a reasonable value
        % Users can always manually resize the dialog as needed.

        %DialogWidth - Width of dialog (in pixels)
        DialogWidth = 450

        %DialogHeight - Height of dialog (in pixels)
        DialogHeight = 250
    end

    properties(SetAccess = private)
        %ROSParameter - Currently selected parameter name
        ROSParameter = ''

        %ROSParameterType - Data type of currently selected parameter
        ROSParameterType = ''

        %ParameterList - List of all available parameter names
        ParameterList = {}      % list of available parameters

        %ParameterTypes - List of data types for available parameters
        %   This will be the same length as ParameterList
        ParameterTypes = {}

        %ValidTypes - List that stores the validity of all parameters
        %   This will be a boolean vector with the same length as
        %   ParameterTypes.
        ValidTypes = []

        %CloseFcnHandle - Function handle that is called on dialog closing
        CloseFcnHandle = function_handle.empty

        %ROSMaster - ROS Master information object
        ROSMaster

        %ShowOKButton - Indicates if OK button should be shown or not
        %   If there are no available parameters, we remove the OK button
        %   from the dialog.
        ShowOKButton = true
    end


    methods
        function obj = ParameterSelector()
            obj.ROSMaster = ros.slros.internal.sim.ROSMaster();
            obj.ROSMaster.verifyReachable();
        end


        function dlg = openDialog(obj, closeFcnHandle)
        % closeFcnHandle: handle to function that takes two arguments
        %   closeFcn(isAcceptedSelection, rosTopic, rosMsgType)
        %      isAcceptedSelection: true of user clicked on 'ok', false
        %        if user clicked on 'cancel' or closed window
        %      rosTopic: last selected ROS topic (string)
        %      rosMsgType: message type of last selected ROS topic (string)

            validateattributes(closeFcnHandle, {'function_handle'}, {'scalar'});
            obj.CloseFcnHandle = closeFcnHandle;

            obj.ROSMaster.verifyReachable();
            [obj.ParameterList, obj.ParameterTypes] = obj.ROSMaster.getParameterNamesTypes();

            % Convert MATLAB parameter types to Simulink types
            obj.ParameterTypes = cellfun(@(x) ros.slros.internal.sim.DataTypes.matlabToSimulinkTypeLabel(x), ...
                                         obj.ParameterTypes, 'UniformOutput', false);

            % Check which types are valid
            obj.ValidTypes = cellfun(@(x) ismember(x, {'int32', 'double', 'boolean', ...
                                ros.slros.internal.sim.DataTypes.SimulinkStringType}), ...
                                     obj.ParameterTypes);

            if numel(obj.ParameterList) == 0 || ~any(obj.ValidTypes)
                % Hide the "OK" button if there are no parameters available
                % or if all parameters have invalid data types
                obj.ShowOKButton = false;
            else
                % At least one parameter is stored on the server
                obj.ShowOKButton = true;
            end

            if numel(obj.ParameterList) == 0
                % Add a token row to the table to let the user know that no
                % parameters are available
                obj.ParameterList{1} = message('ros:slros:paramselector:NoParametersFound').getString;
                obj.ParameterTypes{1} = '';
                obj.ValidTypes(1) = false;
            end

            dlg = DAStudio.Dialog(obj);
        end
    end


    methods
        function tableSelectionChangedCallback(obj, dlg, tag)
        %tableSelectionChangedCallback Called when user selects an item from the table

        % Get selected row and retrieve corresponding parameter names
        % and values.
            row = dlg.getSelectedTableRow(tag);
            obj.ROSParameter = obj.ParameterList{row+1}; % value is zero-based
            obj.ROSParameterType = obj.ParameterTypes{row+1};
        end

        function dlgClose(obj, closeaction)
        % closeaction is 'ok' if user clicked OK
        %                'cancel' if user clicked cancel or closed window
            if ~isempty(obj.CloseFcnHandle)
                isAcceptedSelection = strcmpi(closeaction, 'ok');
                try
                    feval(obj.CloseFcnHandle, isAcceptedSelection, obj.ROSParameter, obj.ROSParameterType);
                catch
                    % Absorb all errors. If they are propagated back to
                    % DDG, this causes MATLAB to crash, (Can't convert to
                    % warnings as they are not displayed either).
                end
            end
        end


        function dlgstruct = getDialogSchema(obj)
        %getDialogSchema Construct the dialog
        %   It only consists of a single table showing parameter names
        %   and associated data types.

        % Main dialog
            dlgstruct.DialogTitle = message('ros:slros:paramselector:DialogTitle').getString;
            dlgstruct.HelpMethod = 'ros.slros.internal.helpview';
            dlgstruct.HelpArgs =  {'rosParamSelectDlg'};  % doc topic id
            dlgstruct.CloseMethod = 'dlgClose';
            dlgstruct.CloseMethodArgs = {'%closeaction'};
            dlgstruct.CloseMethodArgsDT = {'string'};

            % Make this dialog modal wrt to other DDG dialogs
            % (doesn't block MATLAB command line)
            dlgstruct.Sticky = true;

            % Buttons to show on dialog (these are options to pass to DDG,
            % not the final strings, so there is no need to use message
            % catalog)
            if obj.ShowOKButton
                dlgstruct.StandaloneButtonSet =  ...
                    {'Ok', 'Cancel', 'Help'}; % also available: 'Revert', 'Apply'
            else
                % Hide the "OK" button if there are no parameters
                % available.
                dlgstruct.StandaloneButtonSet =  {'Cancel', 'Help'};
            end
            dlgstruct.MinMaxButtons = true;

            % Build the table of parameter names and types
            numParams = numel(obj.ParameterList);
            tabledata = cell(numParams, 2);
            for k = 1:numParams
                isValidDataType = obj.ValidTypes(k);
                if isValidDataType
                    backgroundcolor = [255 255 255];
                else
                    % Gray out rows that have unsupported data types
                    backgroundcolor = [230 230 230];
                end

                tabledata{k,1}.Type  = 'edit';
                tabledata{k,1}.Value = obj.ParameterList{k};
                tabledata{k,1}.Enabled  = isValidDataType;
                tabledata{k,1}.BackgroundColor  = backgroundcolor;

                tabledata{k,2}.Type  = 'edit';
                tabledata{k,2}.Value =  obj.ParameterTypes{k};
                tabledata{k,2}.Enabled  = isValidDataType;
                tabledata{k,2}.BackgroundColor  =  backgroundcolor;
            end

            % Construct the table widget
            mytable.Tag = 'paramtable';
            mytable.Type  = 'table';
            mytable.Name = '';
            mytable.Size  = [numParams 2]; % columns: param name, param type
            mytable.Data = tabledata;
            mytable.Grid  = 1;
            mytable.HeaderVisibility = [0 1];
            mytable.Editable = 0;
            mytable.ColHeader = { ...
                message('ros:slros:paramselector:ColumnParamName').getString() ...
                message('ros:slros:paramselector:ColumnParamType').getString() ...
                                };

            % Make parameter name column stretchable, but keep the type
            % column fixed (since all data type strings are fairly short
            mytable.ColumnStretchable = [1 0];

            % Only allow the user to select rows and call an object method
            % when the selection changes
            mytable.SelectionBehavior = 'row';
            if obj.ShowOKButton
                % Select first element in list that is valid
                paramIdx = find(obj.ValidTypes, 1);
                mytable.SelectedRow = paramIdx - 1;
                obj.ROSParameter = obj.ParameterList{paramIdx};
                obj.ROSParameterType = obj.ParameterTypes{paramIdx};
            else
                % Ensures that no element in the list is selected
                mytable.SelectedRow = numParams;
            end
            mytable.MultiSelect = false;
            mytable.SelectionChangedCallback = @obj.tableSelectionChangedCallback;

            % The dialog only contains the table
            dlgstruct.Items = {mytable};

            % Set size of dialog
            dlgstruct.Geometry = obj.getDialogGeometry;
        end
    end

    methods (Access = private)
        function geometry = getDialogGeometry(obj)
            dlgWidth = obj.DialogWidth;
            dlgHeight = obj.DialogHeight;

            % Center the window on the screen
            ss = get(0, 'ScreenSize');
            screen.Width = ss(3);
            screen.Height = ss(4);

            x = (screen.Width - dlgWidth)/2;
            y = (screen.Height - dlgHeight)/2;

            geometry = [x, y, dlgWidth, dlgHeight];
        end
    end
end
