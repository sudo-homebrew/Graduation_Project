classdef CustomMessageRegistry < handle
    %This function is for internal use only. It may be removed in the future.

    %   Copyright 2019-2021 The MathWorks, Inc.
    properties (Constant, Hidden)
        PREFGROUP = ros.internal.utilities.Constants.PreferenceGroup;
        PREFNAME  = 'CustomMessageMap'
        PREFSRVNAME = 'CustomServiceMap'
        PREFNAMEROS1 = 'CustomMessageMapROS1'
        PREFSRVNAMEROS1 = 'CustomServiceMapROS1'
        PREFACTIONNAMEROS1 = 'CustomActionMapROS1'
        PREFVERSIONNAME = 'CustomMessageMapVersion'
        VERSION   = '1.0'
    end

    properties (Hidden)
        FUNCTIONFILEPATH
        FUNCTIONACCESSPATH
        CLASSFILEPATH
        CLASSACCESSPATH
        ROSVERSION
    end

    properties (Hidden, SetAccess=protected)
        MessageMap %Message map that is saved and retrieved
        ServiceMap %Service map that is saved and retrieved
        ActionMap %Action map that is saved and retrieved
    end

    %% Helpers
    methods (Static)
        %Helper to remove stale items only once
        %Users can call removeStale at any time to cleanup the registry
        function checkOnce(inst, reset)
            persistent Checked;
            if isempty(Checked) || reset
                inst.removeStale();
                Checked = true;
            end
        end

        function checkSrvOnce(inst, reset)
            persistent srvChecked;
            if isempty(srvChecked) || reset
                inst.removeSrvStale();
                srvChecked = true;
            end
        end

        function checkActionOnce(inst, reset)
            persistent actionChecked;
            if isempty(actionChecked) || reset
                inst.removeActionStale();
                actionChecked = true;
            end
        end
    end

    methods (Access=private, Hidden)
        % Get the pref and if the pref is present, get the map from the
        % preference. Also verify that the version is same as expected
        function [validPref, validPrefSrv, validPrefAction] = checkAndGetPref(h)
            validPref = false;
            if isequal(h.ROSVERSION,'ros2')
                prefPresent = ispref(h.PREFGROUP, h.PREFNAME);
                prefStruct = getpref(h.PREFGROUP);
                if prefPresent
                    validPref = isa(prefStruct.CustomMessageMap, 'containers.Map');
                    if validPref
                        h.MessageMap = prefStruct.CustomMessageMap;
                        assert(isequal(h.VERSION, prefStruct.CustomMessageMapVersion));
                    end
                end
            else
                prefPresent = ispref(h.PREFGROUP, h.PREFNAMEROS1);
                prefStruct = getpref(h.PREFGROUP);
                if prefPresent
                    validPref = isa(prefStruct.CustomMessageMapROS1, 'containers.Map');
                    if validPref
                        h.MessageMap = prefStruct.CustomMessageMapROS1;
                        assert(isequal(h.VERSION, prefStruct.CustomMessageMapVersion));
                    end
                end
            end

            validPrefSrv = false;
            if isequal(h.ROSVERSION,'ros2')
                srvPrefPresent = ispref(h.PREFGROUP, h.PREFSRVNAME);
                if srvPrefPresent
                    validPrefSrv = isa(prefStruct.CustomServiceMap, 'containers.Map');
                    if validPrefSrv
                        h.ServiceMap = prefStruct.CustomServiceMap;
                        assert(isequal(h.VERSION, prefStruct.CustomMessageMapVersion));
                    end
                end
            else
                srvPrefPresent = ispref(h.PREFGROUP, h.PREFSRVNAMEROS1);
                if srvPrefPresent
                    validPrefSrv = isa(prefStruct.CustomServiceMapROS1, 'containers.Map');
                    if validPrefSrv
                        h.ServiceMap = prefStruct.CustomServiceMapROS1;
                        assert(isequal(h.VERSION, prefStruct.CustomMessageMapVersion));
                    end
                end
            end

            validPrefAction = false;
            actionPrefPresent = ispref(h.PREFGROUP, h.PREFACTIONNAMEROS1);
            if actionPrefPresent
                validPrefAction = isa(prefStruct.CustomActionMapROS1, 'containers.Map');
                if validPrefAction
                    h.ActionMap = prefStruct.CustomActionMapROS1;
                    assert(isequal(h.VERSION, prefStruct.CustomMessageMapVersion));
                end
            end
        end

        % Save all the preference items
        function savePref(h)
            if isequal(h.ROSVERSION,'ros2')
                setpref(h.PREFGROUP, h.PREFNAME, h.MessageMap);
            else
                setpref(h.PREFGROUP, h.PREFNAMEROS1, h.MessageMap);
            end
            setpref(h.PREFGROUP, h.PREFVERSIONNAME, h.VERSION);
        end

        % Save all the srv preference items
        function saveSrvPref(h)
            if isequal(h.ROSVERSION,'ros2')
                setpref(h.PREFGROUP, h.PREFSRVNAME, h.ServiceMap);
            else
                setpref(h.PREFGROUP, h.PREFSRVNAMEROS1, h.ServiceMap);
            end
            setpref(h.PREFGROUP, h.PREFVERSIONNAME, h.VERSION);
        end

        % Save all the action preference items
        function saveActionPref(h)
            setpref(h.PREFGROUP, h.PREFACTIONNAMEROS1, h.ActionMap);
            setpref(h.PREFGROUP, h.PREFVERSIONNAME, h.VERSION);
        end

        %Helper to create the entry to be used in map
        function ent = createEntry(~, ~, installDir, srcPath, dllPath)
            ent = struct('installDir', installDir, ...
                'srcPath', srcPath, ...
                'dllPath', dllPath, ...
                'Release', version('-release'));
        end

        %removeStale removes stale items
        %called on refresh(reset)
        function removeStale(h)
            keys = h.MessageMap.keys;
            for i = 1:numel(keys)
                ent = h.MessageMap(keys{i});
                if ~isfolder(ent.installDir) || ~isfile(ent.dllPath) || ~isfile(ent.srcPath)
                    h.MessageMap.remove(keys{i});
                    h.savePref();
                end
            end
        end

        function removeSrvStale(h)
            srvKeys = h.ServiceMap.keys;
            for i = 1:numel(srvKeys)
                ent = h.ServiceMap(srvKeys{i});
                if ~isfolder(ent.installDir) || ~isfile(ent.dllPath) || ~isfile(ent.srcPath)
                    h.ServiceMap.remove(srvKeys{i});
                    h.saveSrvPref();
                end
            end
        end

        function removeActionStale(h)
            actionKeys = h.ActionMap.keys;
            for i = 1:numel(actionKeys)
                ent = h.ActionMap(actionKeys{i});
                if ~isfolder(ent.installDir) || ~isfile(ent.dllPath) || ~isfile(ent.srcPath)
                    h.ActionMap.remove(actionKeys{i});
                    h.saveActionPref();
                end
            end
        end
    end

    methods (Static)
        function ret = getInstance(varargin)
            persistent instance
            persistent previousVarargin
            if isempty(instance) || ~isequal(varargin,previousVarargin)
                instance = ros.internal.CustomMessageRegistry(varargin{:});
                previousVarargin = varargin;
            end
            ret = instance;
        end
    end

    methods (Access = private)
        % CustomMessageRegistry
        % CustomMessageRegistry(true) to force a reset (i.e. remove stale)
        function h = CustomMessageRegistry(rosver, reset)
            if nargin < 2
                reset = false;
            end
            h.FUNCTIONFILEPATH = fullfile('m','+ros','+internal',['+' (rosver)],'+custommessages');
            h.FUNCTIONACCESSPATH = ['ros.internal.' [(rosver) '.'] 'custommessages.'];
            %TODO: Switch back to custom message packages g2198971
            h.CLASSFILEPATH = fullfile('m','+ros','+msggen');
            h.CLASSACCESSPATH = 'ros.msggen.';
            h.ROSVERSION = rosver;
            %h.CLASSFILEPATH = fullfile('m','+ros','+custom','+msg');
            %h.CLASSACCESSPATH = 'ros.custom.msg.';
            h.MessageMap = containers.Map(); % an empty map
            h.ServiceMap = containers.Map(); % an empty map
            h.ActionMap = containers.Map(); % an empty map
            h.refresh(reset);
        end
    end

    %% Public
    methods
        % refresh gets the registry again from preference
        % refresh(reset) gets the registry and removes stale items
        function refresh(h, reset)
            [prefPresent, srvPrefPresent, actionPrefPresent] = h.checkAndGetPref();
            if ~prefPresent %let us initialize right away
                h.savePref();
            else
                ros.internal.CustomMessageRegistry.checkOnce(h, reset);
            end

            if ~srvPrefPresent %let us initialize right away
                h.saveSrvPref();
            else
                ros.internal.CustomMessageRegistry.checkSrvOnce(h, reset);
            end

            if ~actionPrefPresent %let us initialize right away
                h.saveActionPref();
            else
                ros.internal.CustomMessageRegistry.checkActionOnce(h, reset);
            end
        end

        % getMessageInfo(type) gets the entry in the map for a given
        % message type
        function customMsgInfo = getMessageInfo(h, type)
            if ~isempty(h.MessageMap) && h.MessageMap.isKey(type)
                customMsgInfo = h.MessageMap(type);
            else
                customMsgInfo = [];
            end
        end

        % getServiceInfo(type) gets the entry in the map for a given
        % service type
        function customSrvInfo = getServiceInfo(h, type)
            if ~isempty(h.ServiceMap) && h.ServiceMap.isKey(type)
                customSrvInfo = h.ServiceMap(type);
            else
                customSrvInfo = [];
            end
        end

        % getActionInfo(type) gets the entry in the map for a given
        % action type
        function customActionInfo = getActionInfo(h, type)
            if ~isempty(h.ActionMap) && h.ActionMap.isKey(type)
                customActionInfo = h.ActionMap(type);
            else
                customActionInfo = [];
            end
        end

        %getMessageList gets the list of messages registered
        function msgList = getMessageList(h)
            msgList = {};
            if ~isempty(h.MessageMap)
                msgTypes = h.MessageMap.keys();
                whichMsgs = cellfun(@(msgInfo) (isfield(msgInfo,'Release') && strcmp(msgInfo.Release, version('-release'))), values(h.MessageMap));
                msgList = msgTypes(whichMsgs);
            end
        end

        %getServiceList gets the list of services registered
        function srvList = getServiceList(h)
            srvList = {};
            if ~isempty(h.ServiceMap)
                srvTypes = h.ServiceMap.keys();
                whichSrvs = cellfun(@(srvInfo) (isfield(srvInfo,'Release') && strcmp(srvInfo.Release, version('-release'))), values(h.ServiceMap));
                srvList = srvTypes(whichSrvs);
            end
        end

        %getActionList gets the list of actions registered
        function actionList = getActionList(h)
            actionList = {};
            if ~isempty(h.ActionMap)
                actionTypes = h.ActionMap.keys();
                whichActions = cellfun(@(actionInfo) (isfield(actionInfo,'Release') && strcmp(actionInfo.Release, version('-release'))), values(h.ActionMap));
                actionList = actionTypes(whichActions);
            end
        end

        %getBinDirList gets all folders containing custom message DLLs
        function dirList = getBinDirList(h)
            msgList = getMessageList(h);
            msgInfoList = cellfun(@(msg) getMessageInfo(h, msg), msgList);
            dirList = arrayfun(@(msgInfo) fileparts(msgInfo.dllPath), ...
                msgInfoList, 'UniformOutput', false);
            dirList = unique(dirList);
        end

        %updateMessageInfo(msgInfo) given a msgInfo (returned from
        %getMessageInfo), if the message is registered as custom, will be
        %overwritten with registry entry
        function msgInfo = updateMessageInfo(h, msgInfo)
            type = [msgInfo.pkgName, '/', msgInfo.msgName];
            if ~isempty(h.MessageMap) && h.MessageMap.isKey(type) && isfield(h.MessageMap(type),'Release') && isequal(h.MessageMap(type).Release,version('-release'))
                ent = h.MessageMap(type);
                msgInfo.custom = true;
                msgInfo.path = ent.dllPath;
                msgInfo.srcPath = ent.srcPath;
                msgInfo.installDir = ent.installDir;
                msgInfo.msgStructGen = [h.FUNCTIONACCESSPATH msgInfo.pkgName '.' lower(msgInfo.msgName(1)) msgInfo.msgName(2:end)];
                msgInfo.msgClassGen = [h.CLASSACCESSPATH msgInfo.pkgName '.' msgInfo.msgName];
                msgInfo.msg1to2ConverterName = [h.FUNCTIONACCESSPATH msgInfo.baseName '_1To2_Converter'];
                msgInfo.msg2to1ConverterName = [h.FUNCTIONACCESSPATH msgInfo.baseName '_2To1_Converter'];
            end
        end

        function msgInfo = updateServiceInfo(h, msgInfo)
            type = [msgInfo.pkgName, '/', msgInfo.msgName];
            if ~isempty(h.ServiceMap) && h.ServiceMap.isKey(type) && isfield(h.ServiceMap(type),'Release') && isequal(h.ServiceMap(type).Release,version('-release'))
                ent = h.ServiceMap(type);
                msgInfo.custom = true;
                msgInfo.path = ent.dllPath;
                msgInfo.srcPath = ent.srcPath;
                msgInfo.installDir = ent.installDir;
                msgInfo.msgStructGen = [h.FUNCTIONACCESSPATH msgInfo.pkgName '.' lower(msgInfo.msgName(1)) msgInfo.msgName(2:end)];
                msgInfo.msgClassGen = [h.CLASSACCESSPATH msgInfo.pkgName '.' msgInfo.srvName];
                msgInfo.msg1to2ConverterName = [h.FUNCTIONACCESSPATH msgInfo.baseName '_1To2_Converter'];
                msgInfo.msg2to1ConverterName = [h.FUNCTIONACCESSPATH msgInfo.baseName '_2To1_Converter'];
            end
        end

        function msgInfo = updateActionInfo(h, msgInfo)
            type = [msgInfo.pkgName, '/', msgInfo.msgName];
            if ~isempty(h.ActionMap) && h.ActionMap.isKey(type) && isfield(h.ActionMap(type),'Release') && isequal(h.ActionMap(type).Release,version('-release'))
                ent = h.ActionMap(type);
                msgInfo.custom = true;
                msgInfo.path = ent.dllPath;
                msgInfo.srcPath = ent.srcPath;
                msgInfo.installDir = ent.installDir;
                msgInfo.msgStructGen = [h.FUNCTIONACCESSPATH msgInfo.pkgName '.' lower(msgInfo.msgName(1)) msgInfo.msgName(2:end)];
                msgInfo.msgClassGen = [h.CLASSACCESSPATH msgInfo.pkgName '.' msgInfo.msgName];
            end
        end

        %updateMessageEntry(type, installDir, srcPath, dllPath) adds an
        %entry if one does not exist or updates the existing entry
        function updateMessageEntry(h, type, installDir, srcPath, dllPath)
            info = h.createEntry(type, installDir, srcPath, dllPath);
            h.MessageMap(type) = info;
            h.savePref();
        end

        function updateServiceEntry(h, type, installDir, srcPath, dllPath)
            info = h.createEntry(type, installDir, srcPath, dllPath);
            h.ServiceMap(type) = info;
            h.saveSrvPref();
        end

        function updateActionEntry(h, type, installDir, srcPath, dllPath)
            info = h.createEntry(type, installDir, srcPath, dllPath);
            h.ActionMap(type) = info;
            h.saveActionPref();
        end

        %removeMessageEntry(type) removes a given message entry
        function removeMessageEntry(h,type)
            if h.MessageMap.isKey(type)
                h.MessageMap.remove(type);
                h.savePref();
            end
        end

        %removeServiceEntry(type) removes a given service entry
        function removeServiceEntry(h,type)
            if h.ServiceMap.isKey(type)
                h.ServiceMap.remove(type);
                h.saveSrvPref();
            end
        end

        %removeActionEntry(type) removes a given action entry
        function removeActionEntry(h,type)
            if h.ActionMap.isKey(type)
                h.ActionMap.remove(type);
                h.saveActionPref();
            end
        end
    end
end

% LocalWords:  DLL DLLs custommessages