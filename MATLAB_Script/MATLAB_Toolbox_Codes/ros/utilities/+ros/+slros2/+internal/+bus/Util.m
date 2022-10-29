classdef Util < ros.slros.internal.bus.Util
%This class is for internal use only. It may be removed in the future.

%BUS.UTIL - Utility functions for working with Simulink buses

%   Copyright 2019-2021 The MathWorks, Inc.


    methods (Static)
        function busName = createBusIfNeeded(rosMsgType, model)
            validateattributes(rosMsgType, {'char'}, {'nonempty'});
            validateattributes(model, {'char'}, {});

            ros.slros.internal.bus.Util.loadROS2LibraryDictionary();
            [busExists, busName] = ros.slros2.internal.bus.Util.checkForBus(rosMsgType);
            if busExists
                return;
            end
            [emptyRosMsg, msgInfo] = ros.slros2.internal.bus.Util.newMessageFromSimulinkMsgType(rosMsgType);
            emptyRosMsg = ros.slros.internal.bus.getBusStructForROSMsg(emptyRosMsg, msgInfo,'ros2');
            ros.slros2.internal.bus.createBusDefnInGlobalScope(emptyRosMsg, model);
        end

        function [busExists,busName] = checkForBus(rosMsgType)
            sec = ros.slros2.internal.bus.Util.getDictionaryDataSection();
            busName = ros.slros2.internal.bus.Util.rosMsgTypeToBusName(rosMsgType);
            busExists = exist(sec,busName);
        end

        function allMsgsMap = getAllMessageInfoMapForModel(model)
            allMsgsMap = ros.slros.internal.bus...
                .getAllMessageInfoMapForModel(model,...
                                              'BusUtilityObject',ros.slros2.internal.bus.Util);
        end

        function msgTypes = getBlockLevelMessageTypesInModel(model)
        %getBlockLevelMessageTypesInModel Get all message types used in model
        %   Please note that this includes any ROS service requests or
        %   response message types.
            validateattributes(model, {'char'}, {'nonempty'});
            blockMsgTypes = ros.slros2.internal.bus.Util.getROS2BlocksInModel(model);
            msgTypes = unique( blockMsgTypes );
        end

        function [topLevelMsgTypes, topLevelSvcTypes, ...
                  pubSubMsgBlockList, paramBlockList, ...
                  imageBlockList, timeBlockList, ...
                  svcCallBlockList, logfileBlockList,writeBlockList] = getROS2BlocksInModel(model)
        %getROSBlocksInModel Get a list of ROS2 blocks and associated
        %message types

            validateattributes(model, {'char'}, {'nonempty'});
            topLevelMsgTypes = {};
            topLevelSvcTypes = {};
            paramBlockList = {};
            timeBlockList = {};

            % Find Publish, Subscribe, and Blank Message
            pubSubMsgBlockList = ...
                ros.slros.internal.bus.Util.listBlocks(model, ...
                                                       ['(' ...
                                                        ros.slros2.internal.block.PublishBlockMask.getMaskType '|' ...
                                                        ros.slros2.internal.block.SubscribeBlockMask.getMaskType '|' ...
                                                        ros.slros2.internal.block.MessageBlockMask.MaskType ...
                                                        ')']);
            if ~isempty(pubSubMsgBlockList)
                topLevelMsgTypes = [topLevelMsgTypes; get_param(pubSubMsgBlockList,'messageType')];
            end

            % Find Read Image Blocks
            imageBlockList = ...
                ros.slros.internal.bus.Util.listBlocks(model, ...
                                                       ros.slros2.internal.block.ReadImageBlockMask.MaskType);

            % Find Logfile Blocks
            logfileBlockList = ...
                ros.slros.internal.bus.Util.listBlocks(model, ...
                                                       ['(' ...
                                                        ros.slros.internal.block.ReadDataBlockMask.MaskType ...
                                                        ' from ROS2 Bag' ...
                                                        ')']);
            if ~isempty(logfileBlockList)
                % Only append messages if message type is set
                logfileMsgTypes = get_param(logfileBlockList,'msgType');
                logfileMsgTypes = logfileMsgTypes(~cellfun(@isempty, logfileMsgTypes));
                topLevelMsgTypes = [topLevelMsgTypes; logfileMsgTypes];
            end

            % Find Service Call Blocks
            svcCallBlockList = ...
                ros.slros.internal.bus.Util.listBlocks(model, ...
                                                       ros.slros2.internal.block.ServiceCallBlockMask.getMaskType);
            if ~isempty(svcCallBlockList)
                % We need two message types for each service.
                topLevelSvcTypes = get_param(svcCallBlockList, 'serviceType');
                topLevelMsgTypes = [topLevelMsgTypes; ...
                                    strcat(topLevelSvcTypes,'Request');
                                    strcat(topLevelSvcTypes,'Response')];
            end

            % Write Point Cloud and Write Image Blocks
            writeBlockList = ...
                ros.slros.internal.bus.Util.listBlocks(model, ...
                                                       ['(' ...
                                                        ros.slros2.internal.block.WriteImageBlockMask.MaskType '|'...
                                                        ros.slros2.internal.block.WritePointCloudBlockMask.MaskType ...
                                                        ')']);
            if ~isempty(writeBlockList)
                topLevelMsgTypes = [topLevelMsgTypes; get_param(writeBlockList,'messageType')];
            end
        end

        function clearSLBusesInGlobalScope(blk)
            % CLEARSLBUSESINGLOBALSCOPE Clear the buses registered in the
            % SL bus dictionary

            if exist('blk','var')
                discardChanges = ~ros.slros.internal.block.CommonMask.isLibraryBlock(blk);
            else
                discardChanges = true;
            end
            if discardChanges
                [~,fileName,fileExt] = fileparts(ros.slros.internal.bus.Util.ROS2DataDict);
                Simulink.data.dictionary.closeAll([fileName,fileExt],'-discard');
            end
        end

        function ret = createMATLABStructFromMsgType(ros2Msg, ros2MsgInfo)
        % CREATEMATLABSTRUCTFROMSGTYPE Creates a MATLAB structure that
        % can be used as Simulink.Bus element type
            ret = struct;
            if (ismember('MessageType',fieldnames(ros2MsgInfo)))
                % add MessageType field
                ret.MessageType = ros2MsgInfo.MessageType;
            elseif (ismember('MLdataType',fieldnames(ros2MsgInfo)))
                switch (ros2MsgInfo.MLdataType)
                  case {'string', 'char'}
                    ret = ros2Msg;
                  otherwise
                    ret = feval(ros2MsgInfo.MLdataType, ros2Msg);
                end
                return;
            end
            thisMsgStruct = ros2Msg;
            thisInfoStruct = ros2MsgInfo;
            thisFieldNames = (fieldnames(thisMsgStruct));
            % Remove MessageType field from use
            thisFieldNames(strcmp(thisFieldNames, 'MessageType')) = [];
            arrSize = size(thisMsgStruct);
            for ii = 1:numel(thisFieldNames)
                for arrIter = 1:arrSize(2)
                    ret(arrIter).(thisFieldNames{ii}) = ros.slros2.internal.bus.Util.createMATLABStructFromMsgType(thisMsgStruct(arrIter).(thisFieldNames{ii}), thisInfoStruct.(thisFieldNames{ii}));
                end
            end
        end

        function registerSLBus(busName, busObj)
        % Register Simulink.Bus in global data dictionary
            sec = ros.slros2.internal.bus.Util.getDictionaryDataSection();
            assignin(sec,busName,busObj);
        end

        function busName = rosMsgTypeToBusName(rosMsgType)
            busName = ros.slros.internal.bus.Util.rosMsgTypeToBusName(rosMsgType, '');
        end

        function [datatype,busName] = rosMsgTypeToDataTypeStr(rosMsgType)
        % This is used wherever a Simulink DataTypeStr is required
        % (e.g., for specifying the output datatype of a Constant block)
        % ** DOES NOT CREATE A BUS **
            busName = ros.slros2.internal.bus.Util.rosMsgTypeToBusName(rosMsgType);
            datatype = ['Bus: ' busName];
        end

        function bus = getBusObjectFromBusName(busName, ~)
            sec = ros.slros2.internal.bus.Util.getDictionaryDataSection();
            bus = evalin(sec, busName);
        end

        function [bus, busName] = getBusObjectFromMsgType(rosMsgType, model)
            busName = ros.slros2.internal.bus.Util.createBusIfNeeded(rosMsgType, model);
            bus = ros.slros2.internal.bus.Util.getBusObjectFromBusName(busName, '');
        end

        function [emptyRosMsg, ros2MsgInfo] = newMessageFromSimulinkMsgType(rosMsgType)
        %newMessageFromSimulinkMsgType Create an empty ROS2 message from
        %message type
            persistent ros2MsgCache__
            if isempty(ros2MsgCache__)
                ros2MsgCache__ = containers.Map;
            end
            if isKey(ros2MsgCache__,rosMsgType)
                ros2MsgInfo = ros2MsgCache__(rosMsgType).info;
                emptyRosMsg = ros2MsgCache__(rosMsgType).message;
            else
                [emptyRosMsg, ros2MsgInfo, ~] = ros.internal.getEmptyMessage(rosMsgType,'ros2');
                ros2MsgCache__(rosMsgType) = struct('message',emptyRosMsg,...
                                                    'info',ros2MsgInfo);
            end
        end

        function createVarlenInfoBusIfNeeded(modelName) %#ok<INUSD>
            busName = ros.slros.internal.bus.Util.VarlengthInfoBusName;
            sec = ros.slros2.internal.bus.Util.getDictionaryDataSection();
            if ~exist(sec,busName)
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

                busObj = Simulink.Bus;
                busObj.Description = '';
                busObj.Elements = elems;
                assignin(sec,busName,busObj);
            end
        end

        % Following methods are used in
        % ros.slros2.internal.sim.ROSMsgToBusStructConverter and
        % ros.slros2.internal.sim.BusStructToROSMsgConverter classes for
        % use with ROS2 bus and message conversions
        function busstruct = convertROSTimeStruct(busstruct, msgType)
            if strcmp(msgType, 'builtin_interfaces/Time')
                busstruct.sec = int32(busstruct.sec);
                busstruct.nanosec = uint32(busstruct.nanosec);
            end
        end

        function tempStruct = getEmptyStringStruct(value)
            tempStruct = struct('data', value);
        end

        function ret = convertStringsToUint8(value)
            ret = uint8(char(value)).'; % note the transpose to column vector
        end

        function ret = convertStringsFromBusToMsg(value)
            ret = reshape(char(value), 1, []); % ensure it is row-vector
        end

        function ret = getLength(rosstruct, prop, info)
            if ismember(prop, info.StringProps)
                ret = strlength(string(rosstruct.(prop)));
            else
                ret = length(rosstruct.(prop));
            end
        end

        function rosMsgInstance = getROSMsgInstance(rosmsg, propertyName, isStringArray, elemInfo)
            getCachedMsgFcn = @ros.slros2.internal.bus.Util.newMessageFromSimulinkMsgType;
            if isStringArray
                % string array (string[]) that is mapped to a std_msgs/String[]
                rosMsgInstance = getCachedMsgFcn('std_msgs/String');
            else
                % regular nested bus
                if isempty(rosmsg.(propertyName))
                    % variable-length array
                    rosMsgInstance = getCachedMsgFcn(elemInfo.MsgType);
                else
                    rosMsgInstance = rosmsg.(propertyName)(1);
                end
            end
        end

        function ret = convertBusstructToROS2Msg(msg, msgInfo, emptyMsg)
        % convertBusstructToROS2Msg Create a ROS 2 message structure
        % given a Simulink Bus structure representation

            getCachedMsgFcn = 'ros.slros2.internal.bus.Util.newMessageFromSimulinkMsgType';
            ret = ros.slros.internal.bus.convertBetweenROSMsgAndSLBus(msg, ...
                                                                      msgInfo, emptyMsg, 'BUS2MSG', getCachedMsgFcn);
        end

        function ret = convertROS2MsgToBusstruct(msg, msgInfo, emptyMsg)
        % convertROS2MsgToBusstruct Create a Simulink Bus MATLAB
        % structure for use with bus-creation from a ROS 2 message (by
        % removing all the constant fields)

            getCachedMsgFcn = 'ros.slros2.internal.bus.Util.newMessageFromSimulinkMsgType';
            ret = ros.slros.internal.bus.convertBetweenROSMsgAndSLBus(msg, ...
                                                                      msgInfo, emptyMsg, 'MSG2BUS', getCachedMsgFcn);
        end

        function [inputBusDataType, slInputBusName, outputBusDataType, slOutputBusName] = ...
                rosServiceTypeToDataTypeStr(rosServiceType)
        %rosServiceTypeToDataTypeStr - Get the data type strings corresponding to service type
        %   Note that there are two buses associated with each service
        %   type: one bus for the request and one bus for the response.

            rosReqMessageType = strcat(rosServiceType, 'Request');
            [inputBusDataType,slInputBusName] = ros.slros2.internal.bus.Util.rosMsgTypeToDataTypeStr(rosReqMessageType);
            rosRespMessageType = strcat(rosServiceType, 'Response');
            [outputBusDataType, slOutputBusName] = ros.slros2.internal.bus.Util.rosMsgTypeToDataTypeStr(rosRespMessageType);
        end

        function ret = extractStringData(val, varargin)
        % extractStringData Extract the 'Data' field from the variable
        % length nested string array.
            isStringArrayProperty = varargin{1};
            if isStringArrayProperty
                ret = {val.data};
            else
                ret = val.data;
            end
        end

        function ret = toStruct(msg, msgType)
        % toStruct Return equivalent structure of message without
        % constant fields get message info. This is equivalent to using
        % the "toStruct" method on ROS 1 message object

            [emptyMsg,msgInfo] = ros.slros2.internal.bus.Util.newMessageFromSimulinkMsgType(msgType);
            ret = ros.slros2.internal.bus.Util.convertROS2MsgToBusstruct(msg, msgInfo, emptyMsg);
        end

        function ret = fromStruct(msgType, busstruct)
        % fromStruct Create an empty ROS 2 message and fill it with the
        % input Simulink Bus structure. This is equivalent to creating
        % an empty message using ''rosmessage'' and calling
        % ''fromStruct'' method of the ROS 1 message object.

            [emptyMsg, msgInfo] = ros.slros2.internal.bus.Util.newMessageFromSimulinkMsgType(msgType);
            ret = ros.slros2.internal.bus.Util.convertBusstructToROS2Msg(busstruct, msgInfo, emptyMsg);
        end

        function ret = isStaticArray(msginfo)
            ret = ros.internal.ros2.MessageUtil.isStaticArray(msginfo);
        end

        function ret = isBoundedArray(msginfo)
            ret = ros.internal.ros2.MessageUtil.isBoundedArray(msginfo);
        end

        function isFixed = isFixedSizeArray(msgType,propName,msginfo)
            isFixed = isfield(msginfo, 'MaxLen') && isfield(msginfo, 'MinLen') && ...
                      isnumeric(msginfo.MaxLen) && (msginfo.MaxLen == msginfo.MinLen) && ...
                      ((msginfo.MaxLen > 1) || ...
                       ros.slros.internal.ROSUtil.isFixedSizeArray(msgType,propName,'ros2'));
        end


        function [ret, isBound] = getBoundedArrayLength(msgType, propertyNames, maxLengths)
            propertyNames = cellstr(propertyNames);
            validateattributes(maxLengths, {'numeric'}, {'positive', 'integer', 'numel', numel(propertyNames)});
            ret = maxLengths;
            isBound = false;
            [~, msginfo]=ros.slros2.internal.bus.Util.newMessageFromSimulinkMsgType(msgType);
            for ii=1:numel(propertyNames)
                propName = propertyNames{ii};
                thisInfo = msginfo.(propName);
                isBound = ros.slros2.internal.bus.Util.isBoundedArray(msginfo);
                if isBound
                    ret(ii) = thisInfo.MaxLen;
                end
            end
        end

        % Check if all fields are constants
        function allConstants = isMessageAllConstants(msgType)
            getMsgFcn = @ros.slros2.internal.bus.Util.newMessageFromSimulinkMsgType;
            persistent ros2AllConstMsgCache__
            if isempty(ros2AllConstMsgCache__)
                ros2AllConstMsgCache__ = containers.Map;
            end
            if isKey(ros2AllConstMsgCache__,msgType)
                allConstants = ros2AllConstMsgCache__(msgType);
            else
                [~, ros2MsgMetaData] = getMsgFcn(msgType);
                allConstants = all(cellfun(@(x)(...
                    evalin('caller',['ros2MsgMetaData.',x,'.constant'])),...
                                           ros2MsgMetaData.MatPath));
                ros2AllConstMsgCache__(msgType) = allConstants;
            end
        end


        function sec = getDictionaryDataSection()
           dict = Simulink.data.dictionary.open(...
               ros.slros.internal.bus.Util.ROS2DataDict);
           sec = getSection(dict,'Design Data');
        end
        
        % Get ROSPropertyLists
        function rosPropList = getROSPropertyList(~,propList)
        %getROSPropertyList For ROS 2 the ROS 2 property list is the
        %same as MATLAB structure members (all are in snake_case)
            rosPropList = propList;
        end

    end

end
