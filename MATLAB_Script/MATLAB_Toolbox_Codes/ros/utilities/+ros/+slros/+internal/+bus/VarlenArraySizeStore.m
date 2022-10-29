classdef VarlenArraySizeStore < handle
%This class is for internal use only. It may be removed in the future.

%VarlenArraySizeStore - Global manager for variable-length array sizes
%   VarlenArraySizeStore is an interface for managing variable-length
%   array information for a single Simulink model (this information is
%   stored with the model in the Model Workspace). This class is aware of
%   ROS message structure and the Simulink model, but does not deal with
%   Simulink buses.
%
%   Methods:
%     updateModel      - Update model workspace with modified array-size
%                       info (dirties the model but does not save it)
%
%     getUserSpecifiedArrayInfo   - Get array-size info for a ROS message type
%     setUserSpecifiedArrayInfo   - Set array-size info for a ROS message type
%     clearUserSpecifiedArrayInfo - Clear array-size info for a ROS message type
%
%     getTruncateAction - Get truncate action for the model
%     setTruncateAction - Set truncate action for the model
%
%     applyMaxLengths  -  Apply stored array-size limits for a message
%                         (if they exist), else apply defaults
%
%   Static Methods:
%     getDefaultTruncateAction - Get default truncate action
%     applyDefaultMaxLengths  - Apply default array-size limits to a
%                               message
%
%   See also: bus.VarLenArrayInfo

%   Copyright 2014-2019 The MathWorks, Inc.


% Outline of what happens when user changes a max-length via Array
% Size Manager (ASM)
%
% 1) Until the user clicks "OK", no changes take effect
%
% 2) When the user clicks "OK", ASM passes the modified values
%    to Store.setUserSpecifiedArrayInfo, invokes
%    Store.updateModel, and then closes the dialog.
%
% 3) Store.setUserSpecifiedArrayInfo() caches the modified values
%    internally (to the Store object) but doesn't do anything else
%
% 4) Store.updateModel() saves the modified values to the model
%    workspace and also clears all buses in the global scope.
%    It also "dirties" the model (i.e., tells Simulink that the
%    model is modified) but *does not save the model to disk*.
%
%    - Since the buses are cleared, any action that requires the bus
%      (e.g., Ctrl+D) will trigger recreation of the bus object,
%      which will get the updated max-length values.
%
% 5) If the user saves the model, the model workspace is saved with
%    the model (essentially, as a MAT file within the slx file). When
%    the model is closed, the buses are cleared from the base
%    workspace. When the user reopens the open and does a Ctrl+D,
%    the buses will get the new (modified) max-length values.
%
%   - If the user closes the model without saving, the modified
%     max-length values are lost (since they've only been saved to
%     the model workspace). The closing of the model also clears the
%     buses. If the user then reopens the model and does a Ctrl+D,
%     the buses will get the original (unmodified) values.
%

    properties(Constant)
        DefaultPrimitiveArrayMaxLen = 128
        DefaultMessageArrayMaxLen = 16
    end

    properties(Constant, Access=private)
        WorkspaceVarName = 'SL_ROS_VariableLengthArrays_MaxSizes'
    end


    properties(SetAccess=private)
        ModelWorkspace
        DataIsModified = false
        ModelWSListener

        ModelName
        MsgTypeMap
        CommonInfo
    end
    
    properties(Access=private)
        BusUtil
    end


    methods
        function obj = VarlenArraySizeStore(model, varargin)
        % No point in constructing without a model
        % we would have to keep checking for validity of Map
            p = inputParser;
            addRequired (p, 'model', @(x)validateattributes(x, {'char'}, {'nonempty'}));
            addParameter(p, 'BusUtilityObject', ros.slros.internal.bus.Util, @(x)isa(x, 'ros.slros.internal.bus.Util'));
            parse(p, model, varargin{:});
            model = p.Results.model;
            obj.BusUtil = p.Results.BusUtilityObject;

            % Check if model is loaded. bdIsLoaded() does not work for
            % parameterized model references, so instead, we do a simple
            % get_param. If the model is not loaded (or the model name is
            % invalid), we throw an error.
            try
                get_param(model, 'Name');
            catch
                % We could use load_system, but it's not clear when we
                % would unload (and there would be no indication to use
                % that the model is loaded). Cleaner to let the user
                % manage the load/unload
                error(message('ros:slros:arraysizestore:UnableToLoadModel', model));
            end

            try
                obj.ModelWorkspace = get_param(model,'ModelWorkspace');
            catch
                % Model does not have a ModelWorkspace param (most likely
                % old-style .mdl file).
                error(message('ros:slros:arraysizestore:SLXFormatRequired', model));
            end

            if isempty(obj.ModelWorkspace)
                % Most likely, the model is a library.
                if strcmpi(get_param(bdroot(model), 'BlockDiagramType'), 'library')
                    error(message('ros:slros:arraysizestore:LibraryNotAllowed', model));
                else
                    error(message('ros:slros:arraysizestore:UnableToLoadWorkspace', model));
                end
            end

            obj.ModelName = bdroot(model);
            obj.ModelWSListener = addlistener(obj.ModelWorkspace, 'ObjectBeingDestroyed', @(~,~) delete(obj));
            obj.loadDataFromWorkspace();
        end


        function updateModel(obj)
            if ~isa(obj.ModelWorkspace,'handle')
                error(message('ros:slros:arraysizestore:ModelNoLongerLoaded', obj.ModelName));
            end

            if obj.DataIsModified
                obj.saveDataToWorkspace();
                % ModelWorkspace.saveToModel() cannot be called if source is
                % 'Model File'. Just set the dirty bit and let the user decide
                % whether to save the model or not
                owningModel = obj.ModelWorkspace.ownerName;
                set_param(owningModel,'Dirty','on');

                % Clear the workspace bus variables (so that
                % subsequent simulation runs get updated sizes)
                obj.BusUtil.clearSLBusesInGlobalScope(obj.ModelName);
            end
        end


        function arrayinfo = getUserSpecifiedArrayInfo(obj, msgType)
        % msgType - ROS message type (e.g., 'std_msgs/Header')
            validateattributes(msgType, {'char'}, {'nonempty'});
            if obj.MsgTypeMap.isKey(msgType)
                msginfoStruct = obj.MsgTypeMap(msgType);
                % resuscitate the VarLenArrayInfo object
                arrayinfo = ros.slros.internal.bus.VarLenArrayInfo(msgType, obj.ModelName, 'BusUtilityObject', obj.BusUtil);
                arrayinfo.initFromStruct(msginfoStruct);
            else
                arrayinfo = [];
            end
        end


        function setUserSpecifiedArrayInfo(obj, msgType, arrayinfo)
        % msgType - ROS message type (e.g., 'std_msgs/Header')
        % arrayinfo - VarLenArrayInfo object with information about the message type

        % There is no corresponding get
            validateattributes(msgType, {'char'}, {'nonempty'});
            validateattributes(arrayinfo, {'ros.slros.internal.bus.VarLenArrayInfo'}, {'scalar'});

            % Don't store as handle array (this is an internal class,
            % whereas this information will be saved with model, and can
            % persist across releases)
            obj.MsgTypeMap(msgType) = arrayinfo.convertToStruct();
            obj.DataIsModified = true;
        end


        function clearUserSpecifiedArrayInfo(obj, msgType)
        % msgType - ROS message type (e.g., 'std_msgs/Header')
            validateattributes(msgType, {'char'}, {'nonempty'});
            if obj.MsgTypeMap.isKey(msgType)
                obj.MsgTypeMap.remove(msgType);
                obj.DataIsModified = true;
            end
        end


        function out = getTruncateAction(obj)
            out = obj.CommonInfo.ArrayTruncateAction;
        end


        function setTruncateAction(obj, action)
            validateattributes(action, ...
                               {'ros.slros.internal.bus.VarLenArrayTruncationAction'}, {'scalar'});

            if obj.CommonInfo.ArrayTruncateAction ~= action
                obj.CommonInfo.ArrayTruncateAction = action;
                obj.DataIsModified = true;
            end
        end


        function isUserSpecified = applyMaxLengths(obj, arrayinfo)
        % applyMaxLengths(ARRAYINFO) applies stored array-size limits
        % for a message (if they exist) to ARRAYINFO. If there are no stored
        % limits, it applies default array-size limits.
        % Note: ARRAYINFO includes information about ROS message type
        %
            validateattributes(arrayinfo, {'ros.slros.internal.bus.VarLenArrayInfo'}, {'scalar'});

            msgtype = arrayinfo.MessageType;
            if obj.MsgTypeMap.isKey(msgtype)
                % msg is in database, return existing values

                % doApplyDefaults will be flipped to false only if all
                % conditions are met
                doApplyDefaults = true;

                storedInfo = obj.MsgTypeMap(msgtype);
                validStoredInfo = isstruct(storedInfo) && ...
                    all(isfield(storedInfo, {'PropertyName', 'DataType', 'MaxLength'}));
                                
                if validStoredInfo
                    storedNames = {storedInfo.PropertyName};
                    curNames = arrayinfo.PropertyNames;
                    if numel(storedNames) == numel(curNames) && all(strcmpi(sort(storedNames), sort(curNames)))
                        arrayinfo.setMaxLength(storedNames, [storedInfo.MaxLength]);
                        doApplyDefaults = false;
                        storedInfoMismatch = false;
                    else % stored property names don't match
                        storedInfoMismatch = true;
                    end
                else % stored info doesn't look right
                    storedInfoMismatch = true;
                end

            else % msg is not in database
                doApplyDefaults = true;
                storedInfoMismatch = false;
            end

            if storedInfoMismatch
                obj.clearUserSpecifiedArrayInfo(msgtype); % remove from store
                warning(message('ros:slros:arraysizestore:MaxArraySizesReset', ...
                                arrayinfo.MessageType, obj.ModelName));
            end

            if doApplyDefaults || storedInfoMismatch
                obj.applyDefaultMaxLengths(arrayinfo, obj.BusUtil);
                isUserSpecified = false;
            else
                isUserSpecified = true;
            end
        end

    end

    %%
    methods(Access=private)

        function loadDataFromWorkspace(obj)
            if ~obj.ModelWorkspace.hasVariable(obj.WorkspaceVarName)
                applyDefaults = true;
            else
                try
                    data =  obj.ModelWorkspace.getVariable(obj.WorkspaceVarName);
                    obj.CommonInfo = data.CommonInfo;
                    % convert from string to enum type
                    obj.CommonInfo.ArrayTruncateAction = ...
                        ros.slros.internal.bus.VarLenArrayTruncationAction.fromChar(data.CommonInfo.ArrayTruncateAction);
                    obj.MsgTypeMap = data.MsgTypeMap;
                    applyDefaults = false;
                catch
                    warning(message('ros:slros:arraysizestore:CorruptedModelWorkspace', ...
                                    obj.ModelName));
                    applyDefaults = true;
                    obj.DataIsModified = true;
                    % Clear workspace bus variables to flush out existing information
                    obj.BusUtil.clearSLBusesInGlobalScope(obj.ModelName);
                end
            end

            if applyDefaults
                obj.CommonInfo = struct();
                obj.CommonInfo.ArrayTruncateAction = obj.getDefaultTruncateAction();
                obj.MsgTypeMap = containers.Map;
            end
        end


        function saveDataToWorkspace(obj)
            commonInfo = obj.CommonInfo;
            commonInfo.ArrayTruncateAction = char(commonInfo.ArrayTruncateAction);
            data = struct('CommonInfo', commonInfo, 'MsgTypeMap', obj.MsgTypeMap);
            obj.ModelWorkspace.assignin(obj.WorkspaceVarName, data);
        end

    end

    %%
    methods(Static)
        function out = getDefaultTruncateAction()
            out = ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning;
        end

        function applyDefaultMaxLengths(arrayinfo, busUtilObj)
            import ros.slros.internal.bus.VarlenArraySizeStore
            validateattributes(arrayinfo, {'ros.slros.internal.bus.VarLenArrayInfo'}, {'scalar'});
            propNames = arrayinfo.PropertyNames;
            for i=1:numel(propNames)
                [~, isROSMsgType] = arrayinfo.getDataType(propNames{i});
                if isROSMsgType
                    % Array of ROS messages
                    maxLength = VarlenArraySizeStore.DefaultMessageArrayMaxLen;
                else
                    % Array of primitive type
                    % Note: strings are mapped to arrays of uint8
                    maxLength = VarlenArraySizeStore.DefaultPrimitiveArrayMaxLen;
                end
                maxLength = busUtilObj.getBoundedArrayLength(arrayinfo.MessageType, propNames{i}, maxLength);
                arrayinfo.setMaxLength(propNames{i}, maxLength);
            end
        end

    end

end
