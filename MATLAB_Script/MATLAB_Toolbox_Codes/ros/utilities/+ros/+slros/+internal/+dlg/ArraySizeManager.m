classdef ArraySizeManager < handle
%This class is for internal use only. It may be removed in the future.

%ArraySizeManager - User-facing dialog for managing variable-length arrays in ROS messages
%   ArraySizeManager opens a DDG dialog that lets the user inspect
%   and modify the maximum sizes of variable-length arrays, for all ROS
%   messages used in a Simulink model. Once the user accepts the
%   changes, the information is written back to the model and the model
%   is dirtied, *but it is not saved*.
%
%   Note: this class does *not* deal with Simulink buses (other than to
%   clear them from global scope) or with the Simulink model. It only
%   interacts with VarlenArraySizeStore and VarLenArrayInfo.
%
%   Sample use:
%    mgr = ros.slros.internal.dlg.ArraySizeManager(gcs);
%    mgr.openDialog
%
%   See also: bus.VarlenArraySizeStore

%   Copyright 2014-2020 The MathWorks, Inc.

    properties(SetAccess=immutable)
        %ModelName - Name of the Simulink model
        ModelName

        %NumMsgTypesInModel - Total number of ROS message types in model
        NumMsgTypesInModel = 0

        %NumVarLenMsgTypesInModel - Total number of ROS message types in
        %   model with variable-length messages
        NumVarLenMsgTypesInModel = 0

        %MsgTypeList - Cell array of all ROS message types in the model
        %   with variable-length messages
        MsgTypeList
    end

    properties(SetAccess=private)
        Dialog
    end

    properties(GetAccess=private, SetAccess=immutable)
        %VarlenSizeStore - VarlenArraySizeStore object (handles
        % saving/retrieving user-specified max lengths). The handle is
        % immutable but object state is modifiable after construction
        VarlenSizeStore

        %MsgVarLenInfo - containers.Map that maps from a ROS message type
        % to corresponding max-length information. The handle is
        % immutable but object state is modifiable after construction
        MsgVarLenInfo
    end

    properties(Access=private)
        Position = [200 200]  % x,y position in pixels
        SelectedMsgTypeIndex
        TruncateAction
        % Bus utility object
        BusUtilObj
    end


    methods

        function obj = ArraySizeManager(modelname)

            modelname = bdroot(modelname);
            obj.ModelName = modelname;

            % Below, we get the maximum length info from
            % Util.getAllMessageInfoMapForModel, which consults the
            % current bus, but uses VarlenArraySizeStore to distinguish
            % between customized vs. default values. This can cause a
            % problem if the bus definitions in the workspace are
            % out-of-sync for any reason (e.g., if the user has modified
            % the bus definitions using BUSEDITOR).
            %
            % The cleanest solution is to clear the buses in the workspace
            % upon entry; they will be recreated in the call to
            % Util.getAllMessageInfoMapForModel with the stored
            % information. (This clearing only needs to be done for the
            % current model but it is safe to clear buses for all models,
            % and also simpler).

            obj.BusUtilObj = ros.slros.internal.bus.Util;
            obj.BusUtilObj.clearSLBusesInGlobalScope(modelname);
            % Get transitive closure for all messages in model
            
            if isequal(exist('ros.slroscpp.internal.bus.Util','class'),8)
                obj.BusUtilObj = ros.slroscpp.internal.bus.Util;
                allMsgsMap = obj.BusUtilObj.getAllMessageInfoMapForModel(modelname);
            else
                allMsgsMap = obj.BusUtilObj.getAllMessageInfoMapForModel(modelname);
            end
            % ArraySizeManager is a common ROS1/ROS2 SL toolstrip App.
            % ArraySizeManager does not have any idea about the contents of
            % this model and it shouldn't. Attempt to search blocks can
            % slow-down opening of App and performance (remember
            % for Simulation we don't need to configure hardware board
            % entry).
            if isequal(exist('ros.slros2.internal.bus.Util','class'),8) && isempty(allMsgsMap)
                obj.BusUtilObj = ros.slros2.internal.bus.Util;
                allMsgsMap = obj.BusUtilObj.getAllMessageInfoMapForModel(modelname);
            end


            obj.VarlenSizeStore = ros.slros.internal.bus.VarlenArraySizeStore(modelname, 'BusUtilityObject', obj.BusUtilObj);

            % Now run over all the messages
            allMsgsTypesInModel = keys(allMsgsMap);
            obj.NumMsgTypesInModel = numel(allMsgsTypesInModel);
            obj.MsgVarLenInfo = containers.Map;

            for i=1:numel(allMsgsTypesInModel)
                msgType = allMsgsTypesInModel{i};

                msginfo = ros.slros.internal.bus.VarLenArrayInfo(msgType, modelname, 'BusUtilityObject', obj.BusUtilObj);
                if msginfo.hasVarLenArrayProperties()
                    % applyMaxLengths will apply user-customizations if present
                    isUserSpecified = obj.VarlenSizeStore.applyMaxLengths(msginfo);
                    obj.MsgVarLenInfo(msgType) = ...
                        struct('MsgTypeInfo', msginfo, 'UserSpecified', isUserSpecified, 'IsDirty', false);
                end

                obj.TruncateAction.CurrentState = obj.VarlenSizeStore.getTruncateAction();
                obj.TruncateAction.IsDirty = false;
            end

            obj.MsgTypeList = sort(keys(obj.MsgVarLenInfo));
            obj.NumVarLenMsgTypesInModel = numel(obj.MsgTypeList);
            obj.SelectedMsgTypeIndex = 1;
            obj.setPositionWrtModelWindow();
            % openDialog() will put up error dialog if there are no
            % messages in the model
        end

        function setPositionWrtModelWindow(obj)
        % There is no way to set the size of the DDG Window (using MCOS)
        % without specifying the location as well. So set the location
        % relative to the model window
            pos = get_param(obj.ModelName, 'Location'); % [x y width height] (in pixels)
                                                        % position the dialog 1/10 of the way from top-left corner
                                                        % When right monitor is primary and model is on left, width
                                                        % can be negative ([-1772 59 -958 589]) - hence using "abs"
            obj.Position = round([pos(1)+ abs(pos(3)/10) pos(2)+ abs(pos(4)/10)]);
        end

        function set.Position(obj, val)
        % the X,Y position for a block can be obtained using get(gcbh,'Position')
        % NOTE: The validation allows "negative" values for the
        % position values, as under situation with dual monitor, the
        % model can be on non-primary monitor (i.e., in the negative
        % direction) and as the ArraySizeManager dialog is placed
        % always relative to the model, it can have negative position
        % values as well.
            validateattributes(val, {'numeric'}, {'size', [1 2], 'real', 'integer'});
            obj.Position = val;
        end

        function openDialog(obj)
            if obj.NumMsgTypesInModel <= 0 || obj.NumVarLenMsgTypesInModel <= 0
                if obj.NumMsgTypesInModel <= 0
                    msgStr = message('ros:slros:arraysizemgr:NoROSMessages', obj.ModelName).getString;
                else
                    msgStr = message('ros:slros:arraysizemgr:NoVarLenROSMessages', obj.ModelName).getString;
                end
                dlgTitle = message('ros:slros:arraysizemgr:DialogTitle').getString;
                dlgProvider = DAStudio.DialogProvider;
                % obj.Dialog = dlgProvider.errordlg(msgStr, dlgTitle, true); % non-blocking
                obj.Dialog = dlgProvider.msgbox(msgStr, dlgTitle, true); % non-blocking
            else
                obj.SelectedMsgTypeIndex = 1;
                obj.Dialog = DAStudio.Dialog(obj);
                obj.Dialog.refresh;
            end
        end

        function tableChangedCallback(obj, dlg, row, col, newValue)
        % Invoked on any changes to the table
            msgdata = obj.MsgVarLenInfo( obj.MsgTypeList{obj.SelectedMsgTypeIndex} );
            msginfo = msgdata.MsgTypeInfo;
            propertyName = dlg.getTableItemValue('rostable', row, 0);

            oldValue = msginfo.getMaxLength(propertyName);

            newValue = max(round(str2double(newValue)), 0);
            % set the maximum length to bounded array value
            boundedArrLen = obj.BusUtilObj.getBoundedArrayLength(msginfo.MessageType, propertyName, newValue);
            if (boundedArrLen < newValue)
                warnTitle = message('ros:slros:arraysizemgr:BoundedArrayWarnDlg').getString;
                warnMsg = message('ros:slros:arraysizemgr:BoundedArrayWarnMsg',propertyName, boundedArrLen).getString;
                warndlg(warnMsg, warnTitle, 'modal');
                newnum = boundedArrLen;
            else
                newnum = newValue;
            end
            % row and col are zero-based
            if isnan(newnum) || ~(newnum > 0) || ~isreal(newnum)
                strValue = num2str(oldValue); % reset back to old value
            else
                % even if new value is valid, may need to modify it
                % (e.g., due to rounding)
                msginfo.setMaxLength(propertyName, newnum);
                strValue =  num2str(newnum);

                msgdata.MsgTypeInfo = msginfo;
                msgdata.IsDirty = true;
                obj.MsgVarLenInfo( obj.MsgTypeList{obj.SelectedMsgTypeIndex} ) = msgdata;
            end
            dlg.setTableItemValue('rostable', row, col, strValue);
            dlg.refresh;
        end

        function msgTypeChangedCallback(obj, dlg, tag, value) %#ok<INUSL>
        % Invoked when user selects another message type from listbox
        % value is the index of the user selection (zero-index).
            try
                if isnumeric(value) && isscalar(value)
                    obj.SelectedMsgTypeIndex = value+1;
                    msgtype = obj.MsgTypeList{obj.SelectedMsgTypeIndex};
                    msgdata = obj.MsgVarLenInfo(msgtype);
                    if msgdata.UserSpecified
                        dlg.setWidgetValue('modifydefaults', 0);
                    else
                        dlg.setWidgetValue('modifydefaults', 1);
                    end
                end
            catch ME
                disp(ME.getReport);
            end

            dlg.refresh;
        end

        function useDefaultsCheckboxCallback(obj, dlg, tag, value) %#ok<INUSL>
        % Invoked when user changes the "use defaults" dropdown
        % <value> is the index of the selection
        %   0 = use default, 1 = specify custom
            try
                msgtype = obj.MsgTypeList{obj.SelectedMsgTypeIndex};
                msgdata = obj.MsgVarLenInfo(msgtype);
                wasUserSpecified = msgdata.UserSpecified;
                msgdata.UserSpecified = (value == 0);
                if wasUserSpecified && ~msgdata.UserSpecified
                    % transitioning from user-specified to default
                    obj.VarlenSizeStore.applyDefaultMaxLengths(msgdata.MsgTypeInfo, obj.BusUtilObj);
                    msgdata.IsDirty = true;
                end

                obj.MsgVarLenInfo(msgtype) = msgdata;
            catch ME
                disp(ME.getReport);
            end
            dlg.refresh;
        end

        function truncateActionDropdownCallback(obj, dlg, tag, value) %#ok<INUSL>
        % Invoked when user changes the truncate action dropdown.
        % <value> is the index of the selection
        %   0 = Truncate w/ warning, 1 = Truncate silently
            try
                if value == 0
                    newState = ...
                        ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning;
                else
                    newState = ...
                        ros.slros.internal.bus.VarLenArrayTruncationAction.DoNothing;
                end

                if obj.TruncateAction.CurrentState ~= newState
                    obj.TruncateAction.CurrentState = newState;
                    obj.TruncateAction.IsDirty = true;
                end

            catch ME
                disp(ME.getReport);
            end
            dlg.refresh;
        end


        function dlgClose(obj, closeaction)
        % closeaction is 'ok' if user clicked OK
        % 'cancel' if user clicked cancel or closed window
            if ~isvalid(obj.VarlenSizeStore)
                % if the user closed the model, then obj.VarlenSizeStore
                % would have deleted itself
                % warn the user & exit
                warning(message('ros:slros:arraysizemgr:ModelAlreadyClosed', obj.ModelName));
                return;
            end

            if ~strcmpi(closeaction, 'ok')
                % Nothing to do if user clicked cancel or closed window
                return;
            end

            try
                modelDirty = false;

                msgTypes = keys(obj.MsgVarLenInfo);
                for i=1:numel(msgTypes)
                    msgdata = obj.MsgVarLenInfo(msgTypes{i});
                    if msgdata.IsDirty
                        modelDirty = true;
                        if msgdata.UserSpecified
                            obj.VarlenSizeStore.setUserSpecifiedArrayInfo(msgTypes{i}, msgdata.MsgTypeInfo);
                        else
                            obj.VarlenSizeStore.clearUserSpecifiedArrayInfo(msgTypes{i});
                        end
                    end
                end

                if obj.TruncateAction.IsDirty
                    modelDirty = true;
                    obj.VarlenSizeStore.setTruncateAction(obj.TruncateAction.CurrentState);
                end

                if modelDirty
                    obj.VarlenSizeStore.updateModel();
                end
            catch ME
                disp(ME.getReport);
                % Absorb all errors. If they are propagated back to
                % DDG, this causes MATLAB to crash, (Can't convert to
                % warnings are not as they are not displayed either).
            end
        end

        function dlgstruct = getDialogSchema(obj)

            assert(obj.NumMsgTypesInModel > 0 && obj.NumVarLenMsgTypesInModel > 0);
            if isempty(obj.SelectedMsgTypeIndex)
                obj.SelectedMsgTypeIndex = 1;
            end

            helptext.Name = message('ros:slros:arraysizemgr:DialogInfo').getString;
            helptext.Type  = 'text';
            helptext.WordWrap = true;
            helptext.Tag = 'rosmsghelp';

            modelname.Name = message('ros:slros:arraysizemgr:ModelName', obj.ModelName).getString();
            modelname.Type  = 'text';
            modelname.WordWrap = false;
            modelname.Bold = true;
            modelname.RowSpan = [1 1];
            modelname.ColSpan = [1 1];
            modelname.Tag = 'modelname';

            modelInfoContainer.Type = 'panel';
            modelInfoContainer.Name = '';
            modelInfoContainer.LayoutGrid = [1 2]; % [numrows numcolumns]
            modelInfoContainer.ColStretch = [1 3]; % [numrows numcolumns]
            modelInfoContainer.Items = {modelname};
            modelInfoContainer.Flat = true;
            modelInfoContainer.Visible = 1;

            truncateAction.Name = message('ros:slros:arraysizemgr:TruncateActionPrompt').getString();
            truncateAction.Type  = 'combobox';
            truncateAction.NameLocation = 1;
            truncateAction.Entries = {...
                message('ros:slros:arraysizemgr:TruncateActionWarn').getString()
                message('ros:slros:arraysizemgr:TruncateActionNone').getString()
                   };
            if obj.TruncateAction.CurrentState == ...
                    ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning
                truncateAction.Value = 0;
            else
                truncateAction.Value = 1;
            end

            truncateAction.Tag = 'truncateaction';
            truncateAction.ObjectMethod = 'truncateActionDropdownCallback'; % call method on UDD source object
            truncateAction.MethodArgs = {'%dialog', '%tag', '%value'}; % '%handle ' is implicit as first arg
            truncateAction.ArgDataTypes = {'handle', 'string', 'mxArray'};
            truncateAction.Alignment = 1; % top-left

            availableMsgs.Name = message('ros:slros:arraysizemgr:MessageTypesPrompt').getString();
            availableMsgs.Type  = 'listbox';
            availableMsgs.RowSpan = [1 2];
            availableMsgs.ColSpan = [1 1];
            availableMsgs.MultiSelect = false;
            availableMsgs.Entries = obj.MsgVarLenInfo.keys();
            availableMsgs.Alignment = 2;
            availableMsgs.Value = obj.SelectedMsgTypeIndex-1; % convert to zero-based index
            availableMsgs.ObjectMethod = 'msgTypeChangedCallback'; % call method on UDD source object
            availableMsgs.MethodArgs = {'%dialog', '%tag', '%value'}; % '%handle ' is implicit as first arg
            availableMsgs.ArgDataTypes = {'handle', 'string', 'mxarray'};
            availableMsgs.Tag = 'availablemsgs';

            msgdata = obj.MsgVarLenInfo( obj.MsgTypeList{obj.SelectedMsgTypeIndex} );
            msginfo = msgdata.MsgTypeInfo;
            varLenArrayProps = msginfo.PropertyNames;
            numVarLenArraysInMsg = numel(varLenArrayProps);
            tabledata = cell(numVarLenArraysInMsg, 3);
            for k=1:numVarLenArraysInMsg
                if msgdata.UserSpecified
                    backgroundcolor = [255 255 255];
                else
                    backgroundcolor = [230 230 230];
                end

                propertyName = varLenArrayProps{k};
                propertyMaxLength = msginfo.getMaxLength(propertyName);
                propertyDataType = msginfo.getDataType(propertyName);
                tabledata{k,1}.Type  = 'edit';
                tabledata{k,1}.Value = propertyName;
                tabledata{k,1}.Enabled  = false;
                tabledata{k,1}.BackgroundColor  = backgroundcolor;

                tabledata{k,2}.Type  = 'edit';
                tabledata{k,2}.Value =  propertyDataType;
                tabledata{k,2}.Enabled  = false;
                tabledata{k,2}.BackgroundColor  =  backgroundcolor;

                tabledata{k,3}.Type  = 'edit';
                tabledata{k,3}.Value = sprintf('%d', propertyMaxLength);
                tabledata{k,3}.BackgroundColor  =  backgroundcolor;
                tabledata{k,3}.Enabled  = msgdata.UserSpecified;
            end

            mytable.Type  = 'table';
            mytable.Name = '';
            mytable.Size  = [numVarLenArraysInMsg 3]; % columns: variable name, type, max length (editable)
            mytable.Data = tabledata;
            mytable.Grid  = 1;
            if msgdata.UserSpecified
                mytable.ReadOnlyColumns = [0 1];
            else
                mytable.ReadOnlyColumns = [0 1 2];
            end
            mytable.HeaderVisibility = [0 1];
            mytable.Editable = 1;
            mytable.ColHeader = {
                message('ros:slros:arraysizemgr:ColumnHdrPropName').getString() ...
                message('ros:slros:arraysizemgr:ColumnHdrPropType').getString() ...
                message('ros:slros:arraysizemgr:ColumnHdrMaxItems').getString()
                                };
            if ismac
                % on MACI automatic size of column headers is not set
                % correctly, so assign the proper column width
                widthCorrection = 4; % number of character-width to correct from the column width
                mytable.ColumnCharacterWidth = cellfun(@(x)length(x), mytable.ColHeader) - widthCorrection;
            end
            mytable.ValueChangedCallback = @obj.tableChangedCallback;
            mytable.Tag = 'rostable';
            mytable.RowSpan = [2 2];
            mytable.ColSpan = [3 4];

            % enabling this also turns makes column headers boldface
            % mytable.SelectionBehavior = 'row';

            useDefaultsCheckbox.Name = message('ros:slros:arraysizemgr:MsgActionUseDefault').getString();
            useDefaultsCheckbox.Type  = 'checkbox';
            useDefaultsCheckbox.RowSpan = [1 1];
            useDefaultsCheckbox.ColSpan = [3 4];
            if msgdata.UserSpecified
                useDefaultsCheckbox.Value = 0;
            else
                useDefaultsCheckbox.Value = 1;
            end
            useDefaultsCheckbox.Tag = 'modifydefaults';
            useDefaultsCheckbox.ObjectMethod = 'useDefaultsCheckboxCallback'; % call method on UDD source object
            useDefaultsCheckbox.MethodArgs = {'%dialog', '%tag', '%value'}; % '%handle ' is implicit as first arg
            useDefaultsCheckbox.ArgDataTypes = {'handle', 'string', 'mxArray'};

            tablecontainer.Name = '';
            tablecontainer.Type = 'group';
            tablecontainer.LayoutGrid = [2 4];
            % tablecontainer.ColStretch = [0 1 1 1 1 1 1];
            tablecontainer.Items = {availableMsgs, useDefaultsCheckbox, mytable};
            tablecontainer.Visible = 1;
            tablecontainer.Alignment = 0;

            % container
            mycontainer.Type = 'panel';
            mycontainer.Name = '';
            mycontainer.Items = {modelInfoContainer, helptext, truncateAction, tablecontainer};
            mycontainer.Visible = 1;

            % Main dialog
            dlgstruct.DialogTitle = message('ros:slros:arraysizemgr:DialogTitle').getString;
            dlgstruct.HelpMethod = 'ros.slros.internal.helpview';
            dlgstruct.HelpArgs =  {'rosManageSizesDlg'}; % doc topic id
            dlgstruct.CloseMethod = 'dlgClose';
            dlgstruct.CloseMethodArgs = {'%closeaction'};
            dlgstruct.CloseMethodArgsDT = {'string'};
            % dlgstruct.Sticky = true; % make this dialog modal wrt to other DDG dialogs (doesn't block MATLAB command line)

            % Buttons to show on dialog (these are options to pass to DDG,
            % not the final strings, so there is no need to use message
            % catalog)
            dlgstruct.StandaloneButtonSet =  ...
                {'Ok', 'Cancel', 'Help'}; % also available: 'Revert', 'Apply'

            dlgstruct.Items = {mycontainer};
            dlgstruct.Geometry = [obj.Position  750  450]; % xpos ypos width height
            dlgstruct.DialogTag = ros.slros.internal.dlg.ArraySizeManager.getDialogTag(obj.ModelName);
        end
    end

    methods(Static)

        function launch(modelname)
        % Convenience function for opening the dialog
            mgr = ros.slros.internal.dlg.findAndBringToFront('ros.slros.internal.dlg.ArraySizeManager', ...
                                                             ros.slros.internal.dlg.ArraySizeManager.getDialogTag(modelname), ...
                                                             @(x)isequal(x.ModelName,modelname));
            if isempty(mgr)
                mgr = ros.slros.internal.dlg.ArraySizeManager(modelname);
                mgr.openDialog();
            end
        end

        function tag = getDialogTag(modelName)
            tag = ['slros_arraysizemgr_' modelName];
        end

    end
end
