classdef Util
%This class is for internal use only. It may be removed in the future.

%BUS.UTIL - Utility functions for working with Simulink buses

%   Copyright 2014-2022 The MathWorks, Inc.

    properties(Constant)
        BusNamePrefix = 'SL_Bus_'
        VarlengthInfoBusName = 'SL_Bus_ROSVariableLengthArrayInfo'

        %TimePackage - Pseudo ROS package for Time and Duration structures
        TimePackage = 'ros_time'

        %TimeMessage - Message name for time data in TimePackage
        TimeMessage = 'Time'

        %DurationMessage - Message name for duration data in TimePackage
        DurationMessage = 'Duration'

        %TimeMessageType - Full pseudo message type for time
        %   Note that ROS does not have a dedicated message type for time,
        %   but it uses a standard structure. From the Simulink perspective
        %   each time structure corresponds to a pseudo message type, just
        %   to keep uniform code.
        TimeMessageType = [ros.slros.internal.bus.Util.TimePackage '/' ...
                           ros.slros.internal.bus.Util.TimeMessage]

        %DurationMessageType - Full pseudo message type for duration
        %   Note that ROS does not have a dedicated message type for duration,
        %   but it uses a standard structure. From the Simulink perspective
        %   each duration structure corresponds to a pseudo message type, just
        %   to keep uniform code.
        DurationMessageType = [ros.slros.internal.bus.Util.TimePackage '/' ...
                               ros.slros.internal.bus.Util.DurationMessage]

        %ROS Data dictionary
        ROSDataDict = fullfile(toolboxdir('ros'),'slroscpp','robotlib.sldd');

        %ROS 2 Data dictionary
        ROS2DataDict = fullfile(toolboxdir('ros'),'slros2','ros2lib.sldd');
    end

    %% Utility functions related to variable-length arrays
    methods (Static)

        function out = setMaxLength(modelName, messageType, prop, newLength)
            store = ros.slros.internal.bus.VarlenArraySizeStore(modelName);
            if ismember(prop(1),'A':'Z')
                % ROS message properties always start with upper case
                arrayInfo = ros.slros.internal.bus.VarLenArrayInfo(messageType, modelName);
            else
                arrayInfo = ros.slros.internal.bus.VarLenArrayInfo(messageType, modelName, 'BusUtilityObject', ros.slros2.internal.bus.Util);
            end
            store.applyMaxLengths(arrayInfo);
            arrayInfo.setMaxLength(prop, newLength);
            store.setUserSpecifiedArrayInfo(messageType, arrayInfo);
            store.updateModel();
            out = message('ros:slros:busconvert:ArrayLengthIncreased', ...
                          newLength).getString;
        end

        function [datatype,busName] = varlenInfoBusDataTypeStr()
            busName = ros.slros.internal.bus.Util.VarlengthInfoBusName;
            datatype = ['Bus: ' busName];
        end


        function s = getVarlenInfoStruct(curlen, recvdlen)
            s.CurrentLength = uint32(curlen);
            s.ReceivedLength = uint32(recvdlen);
        end


        function createVarlenInfoBusIfNeeded(modelName)
            busName = ros.slros.internal.bus.Util.VarlengthInfoBusName;
            busExists = ros.slros.internal.existsInGlobalScope(modelName, busName);
            if ~busExists
                elems(1) = Simulink.BusElement;
                elems(1).Name = 'CurrentLength';
                elems(1).Dimensions = 1;
                elems(1).DimensionsMode = 'Fixed';
                elems(1).DataType = 'uint32';
                elems(1).Complexity = 'real';
                elems(1).SamplingMode = 'Sample based';
                elems(1).Description = '';

                elems(2) = Simulink.BusElement;
                elems(2).Name = 'ReceivedLength';
                elems(2).Dimensions = 1;
                elems(2).DimensionsMode = 'Fixed';
                elems(2).DataType = 'uint32';
                elems(2).Complexity = 'real';
                elems(2).SamplingMode = 'Sample based';
                elems(2).Description = '';

                bus = Simulink.Bus;
                bus.Description = '';
                bus.Elements = elems;
                ros.slros.internal.assigninGlobalScope(modelName, busName, bus);
            end
        end


        function infostruct = getArrayInfoStructMetadata()
            infostruct.PropertySuffix = '_SL_Info';
            infostruct.CurLengthProp = 'CurrentLength';
            infostruct.RcvdLengthProp = 'ReceivedLength';
            infostruct.LengthTypeSL = 'uint32';
            infostruct.LengthTypeCpp = 'uint32_T';
        end


        function arrayInfoElemName = getArrayInfoElementName(arrayElemName)
            infostruct = ros.slros.internal.bus.Util.getArrayInfoStructMetadata();
            arrayInfoElemName = [arrayElemName infostruct.PropertySuffix];
        end
    end


    %%  Utility functions for handling std_msgs/Empty
    methods (Static)

        function name = getDummyElementNameForStdEmptyMsg()
            name = 'SL_DummyData';
        end


        function elem = getDummyElementForStdEmptyMsg()
            elem = Simulink.BusElement;
            elem.Name = ros.slros.internal.bus.Util.getDummyElementNameForStdEmptyMsg;
            elem.Dimensions = 1;
            elem.DimensionsMode = 'Fixed';
            elem.DataType = 'boolean';
            elem.Complexity = 'real';
            elem.SamplingMode = 'Sample based';
            elem.Description = '';
        end

    end

    %%  General Bus-related utilities
    methods (Static)
        function allMsgsMap = getAllMessageInfoMapForModel(model)
            allMsgsMap = ros.slros.internal.bus...
                .getAllMessageInfoMapForModel(model);
        end

        function msgTypes = getBlockLevelMessageTypesInModel(model)
        %getBlockLevelMessageTypesInModel Get all message types used in model
        %   Please note that this includes any ROS service requests or
        %   response message types.
            validateattributes(model, {'char'}, {'nonempty'});
            blockMsgTypes = ros.slros.internal.bus.Util.getROSBlocksInModel(model);
            msgTypes = unique( blockMsgTypes );
        end

        function [topLevelMsgTypes, topLevelSvcTypes, ...
                  pubSubMsgBlockList, paramBlockList, ...
                  imageBlockList, timeBlockList, ...
                  svcCallBlockList, logfileBlockList, writeBlockList] = getROSBlocksInModel(model)
        %getROSBlocksInModel Get a list of ROS blocks and associated message types
        %   Note that the entries in TOPLEVELMSGTYPES and TOPLEVELSVCTYPES
        %   might not be unique.

            validateattributes(model, {'char'}, {'nonempty'});

            topLevelMsgTypes = {};
            topLevelSvcTypes = {};

            % Find Publish, Subscribe, and Blank Message
            pubSubMsgBlockList = ...
                ros.slros.internal.bus.Util.listBlocks(model, ...
                                                       ['(' ...
                                                        ros.slros.internal.block.PublishBlockMask.MaskType, '|', ...
                                                        ros.slros.internal.block.SubscribeBlockMask.MaskType, '|', ...
                                                        ros.slros.internal.block.MessageBlockMask.MaskType...
                                                        ')']);
            if ~isempty(pubSubMsgBlockList)
                topLevelMsgTypes = [topLevelMsgTypes; get_param(pubSubMsgBlockList,'messageType')];
            end

            % Find Parameter Blocks
            paramBlockList = ...
                ros.slros.internal.bus.Util.listBlocks(model, ...
                                                       ['(' ...
                                                        ros.slros.internal.block.SetParameterBlockMask.MaskType, '|', ...
                                                        ros.slros.internal.block.GetParameterBlockMask.MaskType ...
                                                        ')']);

            % Find Read Image Blocks
            imageBlockList = ...
                ros.slros.internal.bus.Util.listBlocks(model, ...
                                                       ros.slros.internal.block.ReadImageBlockMask.MaskType);

            % Find Current Time Blocks
            timeBlockList = ...
                ros.slros.internal.bus.Util.listBlocks(model, ...
                                                       ros.slros.internal.block.CurrentTimeBlockMask.MaskType);
            hasBusOutput = cellfun(@(block) strcmp(get_param(block, 'OutputFormat'), 'bus'), timeBlockList);
            if any(hasBusOutput)
                topLevelMsgTypes = [topLevelMsgTypes; ros.slros.internal.bus.Util.TimeMessageType];
            end

            % Find Service Call Blocks
            svcCallBlockList = ...
                ros.slros.internal.bus.Util.listBlocks(model, ...
                                                       ros.slros.internal.block.ServiceCallBlockMask.MaskType);

            if ~isempty(svcCallBlockList)
                % We need two message types for each service.
                topLevelSvcTypes = get_param(svcCallBlockList, 'serviceType');
                topLevelMsgTypes = [topLevelMsgTypes; ...
                                    strcat(topLevelSvcTypes,'Request');
                                    strcat(topLevelSvcTypes,'Response')];
            end

            % Find Logfile Blocks
            logfileBlockList = ...
                ros.slros.internal.bus.Util.listBlocks(model, ...
                                                       ['(' ...
                                                        ros.slros.internal.block.ReadDataBlockMask.MaskType ...
                                                        ' from ROS Bag' ...
                                                        ')']);
            if ~isempty(logfileBlockList)
                % Only append messages if message type is set
                logfileMsgTypes = get_param(logfileBlockList,'msgType');
                logfileMsgTypes = logfileMsgTypes(~cellfun(@isempty, logfileMsgTypes));
                topLevelMsgTypes = [topLevelMsgTypes; logfileMsgTypes];
            end

            % Write Point Cloud and Write Image Blocks
            writeBlockList = ...
                ros.slros.internal.bus.Util.listBlocks(model, ...
                                                       ['(' ...
                                                        ros.slros.internal.block.WriteImageBlockMask.MaskType '|'...
                                                        ros.slros.internal.block.WritePointCloudBlockMask.MaskType...
                                                        ')']);
            if ~isempty(writeBlockList)
                topLevelMsgTypes = [topLevelMsgTypes; get_param(writeBlockList,'messageType')];
            end

        end

        function blockList = listBlocks(model, maskType)
        %listBlocks List blocks of a specific mask type in a model
        %   Note that MASKTYPE can contain regular expressions, e.g.
        %   'ROS (Publish|Subscribe)'.
        %   BLOCKLIST is returned as a column vector

            lbdata = libinfo(bdroot(model), ...
                             'LookUnderMasks', 'all', ...
                             'Regexp', 'on', ...
                             'MaskType', maskType);
            blockList = {lbdata(:).Block};
            blockList = blockList(:);
        end


        function clearSLBusesInGlobalScope(model)
            ros.slros.internal.evalinGlobalScope(model, ['clear ' ros.slros.internal.bus.Util.BusNamePrefix '*']);
        end


        function bus = getBusObjectFromBusName(busName, model)
            bus = ros.slros.internal.evalinGlobalScope(model, busName);
        end


        function [bus,busName] = getBusObjectFromMsgType(rosMsgType, model)
            busName = ros.slros.internal.bus.Util.createBusIfNeeded(rosMsgType, model);
            bus = ros.slros.internal.evalinGlobalScope(bdroot(model), busName);
        end


        function busname = getBusNameFromDataTypeStr(dataTypeStr)
            matches = regexp(dataTypeStr, 'Bus:[ ]\s*(.*)', 'tokens');
            if ~isempty(matches)
                busname = matches{1}{1};
            else
                busname = '';
            end
        end


        function [busExists,busName] = checkForBus(rosMsgType, model)
            busName = ros.slros.internal.bus.Util.rosMsgTypeToBusName(rosMsgType, model);
            busExists = ros.slros.internal.existsInGlobalScope(bdroot(model), busName);
        end


        function busName = createBusIfNeeded(rosMsgType, model)
            validateattributes(rosMsgType, {'char'}, {'nonempty'});
            validateattributes(model, {'char'}, {});

            ros.slros.internal.bus.Util.loadROSLibraryDictionary();
            [busExists,busName] = ros.slros.internal.bus.Util.checkForBus(rosMsgType, model);
            if busExists
                return;
            end

            % If message class does not exist, rosmessage will error
            % (ros:mlros:message:NoMatlabClass).
            emptyRosMsg = ros.slros.internal.bus.Util.newMessageFromSimulinkMsgType(rosMsgType);

            % Workaround for wrong dimensions on some message types with
            % C++ back-end. This should be resolved with g2168773.
            if isequal(rosMsgType,'sensor_msgs/PointCloud2')
                emptyRosMsg.Fields = emptyRosMsg.Fields.empty(0,1);
            end
            ros.slros.internal.bus.createBusDefnInGlobalScope(emptyRosMsg, model);
        end


        %%
        function [datatype, busName, msgType] = rosTimeToDataTypeStr(rosClassName, model)
            msgType = ros.slros.internal.bus.Util.rosTimeToQuasiMsgType(rosClassName);
            [datatype, busName] = ros.slros.internal.bus.Util.rosMsgTypeToDataTypeStr(msgType, model);
        end


        function quasiMsgType = rosTimeToQuasiMsgType(rosClassName)
            entity = ros.slros.internal.ROSUtil.getTimeEntityType(rosClassName);
            if ~isempty(entity)
                quasiMsgType = [ros.slros.internal.bus.Util.TimePackage ...
                                '/' entity]; % can be Time or Duration
            else
                assert(false, 'Unexpected classname: %s', rosClassName);
            end
        end

        function rosMsg = newMessageFromSimulinkMsgType(rosMsgType)
        %newMessageFromSimulinkMsgType Create a new ROS message from message type
        %   Please note that ROSMSGTYPE could refer to a standard ROS
        %   message type, or to a pseudo-message type used only in
        %   Simulink, for example time and duration types.

            if strcmp(rosMsgType, ros.slros.internal.bus.Util.TimeMessageType)
                rosMsg = ros.msg.Time;
            elseif strcmp(rosMsgType, ros.slros.internal.bus.Util.DurationMessageType)
                rosMsg = ros.msg.Duration;
            else
                rosMsg = rosmessage(rosMsgType);
            end
        end


        %%
        function [datatype,busName] = rosMsgTypeToDataTypeStr(rosMsgType, model)
        % This is used wherever a Simulink DataTypeStr is required
        % (e.g., for specifying the output datatype of a Constant block)
        % ** DOES NOT CREATE A BUS **
            busName = ros.slros.internal.bus.Util.rosMsgTypeToBusName(rosMsgType, model);
            datatype = ['Bus: ' busName];
        end


        function busName = rosMsgTypeToBusName(rosMsgType, model)
        %
        % rosMsgTypeToBusName(MSGTYPE,MODEL) returns the bus name
        % corresponding to a ROS message type MSGTYPE (e.g.,
        % 'std_msgs/Int32') and a Simulink model MODEL. The function
        % uses the following rules:
        %
        % Rule 1 - Generate a name using the format:
        %    SL_Bus_<modelname>_<msgtype>
        %
        % Rule 2 - If the result of Rule 1 is longer than 60
        % characters, use the following general format:
        %    SL_Bus_<modelname(1:25)>_<msgtype(end-25:end)>_<hash>
        % where <hash> is a base 36 hash of the full name (output of
        % rule #1).
        %
        % ** THIS FUNCTION DOES NOT CREATE A BUS OBJECT **

            validateattributes(rosMsgType, {'char'}, {'nonempty'});
            assert(ischar(model));

            if isempty(model)
                modelnameSuffix = '';
            else
                modelnameSuffix = '_';
            end

            maxlen = 60; choplen=25;
            assert(maxlen <= namelengthmax);

            busName = [ros.slros.internal.bus.Util.BusNamePrefix ...
                       model modelnameSuffix rosMsgType];
            if length(busName) < maxlen
                busName = matlab.lang.makeValidName(busName, 'ReplacementStyle', 'underscore');
            else
                % add a trailing hash string (5-6 chars) to make the busname unique
                hashStr = ros.slros.internal.bus.Util.hashString(busName);

                idx = strfind(rosMsgType, '/');
                if isempty(idx)
                    idx = 0;
                else
                    idx = idx(1);
                end
                model = model(1:min(end,choplen));  % get first 25 chars
                rosMsgType = rosMsgType(idx+1:min(idx+choplen,end));  % get first 25 chars
                busName = matlab.lang.makeValidName(...
                    [ros.slros.internal.bus.Util.BusNamePrefix ...
                     model modelnameSuffix rosMsgType '_' hashStr], ...
                    'ReplacementStyle', 'underscore');
            end
        end

        function [srvReqBusName, srvRespBusName] = rosServiceTypeToBusNames(rosSrvType, model)
        %rosServiceTypeToBusNames - Get the bus names corresponding to service type
        %   Note that there are two buses associated with each service
        %   type: one bus for the request and one bus for the response.

            srvReqBusName = ros.slros.internal.bus.Util.rosMsgTypeToBusName(strcat(rosSrvType,'Request'), model);
            srvRespBusName = ros.slros.internal.bus.Util.rosMsgTypeToBusName(strcat(rosSrvType,'Response'), model);
        end

        function [srvReqBusType, srvReqBusName, srvRespBusType, srvRespBusName] = ...
                rosServiceTypeToDataTypeStr(rosSrvType, model)
        %rosServiceTypeToDataTypeStr - Get the data type strings corresponding to service type
        %   Note that there are two buses associated with each service
        %   type: one bus for the request and one bus for the response.

            rosReqMessageType = strcat(rosSrvType, 'Request');
            [srvReqBusType,srvReqBusName] = ros.slros.internal.bus.Util.rosMsgTypeToDataTypeStr(rosReqMessageType, model);
            rosRespMessageType = strcat(rosSrvType, 'Response');
            [srvRespBusType, srvRespBusName] = ros.slros.internal.bus.Util.rosMsgTypeToDataTypeStr(rosRespMessageType, model);
        end


        % Following methods are used in
        % ros.slros.internal.sim.ROSMsgToBusStructConverter and
        % ros.slros.internal.sim.BusStructToROSMsgConverter classes to
        % for ROS1 specific bus and message conversions
        function busstruct = convertROSTimeStruct(busstruct, msgType)
            if strcmp(msgType, 'ros_time/Time')
                busstruct.Sec = double(busstruct.Sec);
                busstruct.Nsec = double(busstruct.Nsec);
            end
        end

        function tempStruct = getEmptyStringStruct(value)
            tempStruct = struct('Data', value);
        end

        function ret = convertStringsToUint8(value)
            ret = uint8(value).'; % note the transpose to column vector
        end

        function ret = convertStringsFromBusToMsg(value)
            ret =  reshape(char(value), 1, []); % note the transpose to column vector
        end

        function busstruct = permuteBusElements(busstruct, reorderInfo)
        % Permute the order of properties in busstruct to the expected order in the
        % Simulink bus (e.g., to put the SL_Info properties next to the
        % corresponding variable-length arrays).
            values   = struct2cell(busstruct);
            perm = reorderInfo.permutation;

            f = fields(busstruct);
            newperm = perm;
            for i = 1:length(reorderInfo.augmentedPropertyList)
                newperm(i) = find(ismember(f, reorderInfo.augmentedPropertyList(perm(i))));
            end

            % Reorder struct fields to match bus definition
            % Account for newly added fields that aren't accounted for in reordering
            % information structure (perm will be empty in this case)
            % e.g. std_msgs/Empty dummy data
            if ~isempty(perm)
                busstruct = cell2struct(values(newperm), reorderInfo.augmentedPropertyList(perm), 1);
            end
        end

        function ret = getLength(rosstruct, prop, ~)
            ret = length(rosstruct.(prop));
        end

        function rosMsgInstance = getROSMsgInstance(rosmsg, propertyName, isStringArray, elemInfo) %#ok<INUSD>
            if isStringArray
                % string array (string[]) that is mapped to a std_msgs/String[]
                rosMsgInstance = ros.slros.internal.ROSUtil.getStdStringObj();
            else
                % regular nested bus
                if isempty(rosmsg.(propertyName))
                    % variable-length array
                    rosMsgInstance = feval(class(rosmsg.(propertyName)));
                else
                    rosMsgInstance = rosmsg.(propertyName)(1);
                end
            end

        end

        function ret = toStruct(msg, ~)
        % TOSTRUCT Return equivalent structure for ROS message using the
        % ''toStruct'' method of ROS.Message class

            if isstruct(msg)
                % already converted to struct
                ret = msg;
            else
                ret = toStruct(msg);
            end
        end

        function rosmsg = fromStruct(msgType, busstruct)
        % FROMSTRUCT Create an empty ROS 2 message and fill it with the
        % input Simulink Bus structure.

            rosmsg = rosmessage(msgType); % blank message

            % call the 'fromStruct' method of ROS.Message class to load
            % from a structure
            rosmsg.fromStruct(busstruct);
        end

        function [ret, isBoundedArray] = getBoundedArrayLength(~, ~, defaultVal)
            isBoundedArray = false;
            ret = defaultVal;
        end

        function updateBusSignalNames(model,varargin)
            allBusSelectors = find_system(model, 'FollowLinks', 'on', 'BlockType', 'BusSelector');
            fcn = varargin{1};
            for jj = 1:numel(allBusSelectors)
                fixSignalsInBusSelectors(allBusSelectors{jj},fcn);
            end
            allBusAssigns = find_system(model, 'FollowLinks', 'on', 'BlockType', 'BusAssignment');
            for jj = 1:numel(allBusAssigns)
                fixSignalsInBusAssigns(allBusAssigns{jj}, fcn);
            end
            function fixSignalsInBusAssigns(block, fcn)
                sigs = regexp(get_param(block, 'AssignedSignals'), ',','split');
                sigNames = cellfun(@(x)feval(fcn, x),sigs, 'UniformOutput',false);
                set_param(block,'AssignedSignals',strjoin(sigNames,','));
            end
            function fixSignalsInBusSelectors(block, fcn)
                sigs = regexp(get_param(block, 'OutputSignals'), ',','split');
                sigNames = cellfun(@(x)feval(fcn, x),sigs, 'UniformOutput',false);
                set_param(block,'OutputSignals',strjoin(sigNames,','));
            end
        end

        function ret = extractStringData(val, varargin)
        % EXTRACTSTRINGDATA Extract the 'Data' field from the variable
        % length nested string array.

            ret = {val.Data};
        end

        function ret = isMessageAllConstants(~)
            ret = false;
        end

        function out = getBusElemNameFromROSProp(val, ~)
        % For ROS (class) bus elements and ROS Property are both
        % CamelCase
            out = val;
        end

        function out = getROSPropertyFromBusName(val)
        % For ROS (class) bus elements and ROS Property are both
        % CamelCase
            out = val;
        end

        function sec = getDictionaryDataSection()
            sec = getSection(Simulink.data.dictionary.open(...
                ros.slros.internal.bus.Util.ROSDataDict),'Design Data');
        end

        function len = getCurrentFieldLength(msgType,fieldName,rosVer)
        % Return the current length of specific field

            if isequal(rosVer,'ros')
                msgInfo = ros.slros.internal.bus.VarLenArrayInfo(msgType,bdroot(gcb));
            else
                msgInfo = ros.slros.internal.bus.VarLenArrayInfo(msgType,bdroot(gcb),'BusUtilityObject',ros.slros2.internal.bus.Util);
            end
            len = msgInfo.getMaxLength(fieldName);
        end

        function loadROS2LibraryDictionary()
            % LOADROS2LIBRARYDICTIONARY To workaround a Simulink Library
            % Dictionary data linkage issue, call the dictionary refresh
            % commands below on Simulink load time.
            persistent ros2DictionaryLoaded__
            if isempty(ros2DictionaryLoaded__)
                ros2DictionaryLoaded__ = false;
            end
            if ~ros2DictionaryLoaded__
                Simulink.LibraryDictionary.refresh(fullfile(matlabroot,'toolbox','ros','slros2','ros2lib.slx'));
        		Simulink.LibraryDictionary.getInstanceOfDependencyInfo();
                ros2DictionaryLoaded__ = true;
            end
        end

        function loadROSLibraryDictionary()
            % LOADROSLIBRARYDICTIONARY To workaround a Simulink Library
            % Dictionary data linkage issue, call the dictionary refresh
            % commands below on Simulink load time.
            persistent rosDictionaryLoaded__
            if isempty(rosDictionaryLoaded__)
                rosDictionaryLoaded__ = false;
            end
            if ~rosDictionaryLoaded__
                Simulink.LibraryDictionary.refresh(fullfile(matlabroot,'toolbox','ros','slroscpp','robotlib.slx'));
        		Simulink.LibraryDictionary.getInstanceOfDependencyInfo();
                rosDictionaryLoaded__ = true;
            end
        end
    end

    methods(Static)
        function hashString = hashString(stringIn)
        % Re-implement hashCode generation for backwards compatibility of Bus names

        % For reference: https://stackoverflow.com/questions/15518418
            hashCode = 0;
            nChar = int32(strlength(stringIn));
            for iter = 1:nChar
                hashCode = overflowSum(overflowProduct(hashCode, 31), int32(stringIn(iter)));
            end
            hashString = lower(dec2base(abs(hashCode), 36));

            function out= overflowProduct(a, b)
                out = 0;
                for k = 1:b
                    out = overflowSum(out, a);
                end
            end

            function c = overflowSum(a, b)
                if (a > 0) && (b > (intmax-a))
                    c = intmin + b-(intmax-a)-1;
                elseif (a < 0) && (b < (intmin-a))
                    c = intmax + b-(intmin-a-1);
                else
                    c = a + b;
                end
            end
        end
    end

end
