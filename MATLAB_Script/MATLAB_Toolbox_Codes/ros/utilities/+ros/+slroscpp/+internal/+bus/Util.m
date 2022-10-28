classdef Util < ros.slros.internal.bus.Util
%This class is for internal use only. It may be removed in the future.

%BUS.UTIL - Utility functions for working with Simulink buses

%   Copyright 2019-2021 The MathWorks, Inc.


    methods (Static)
        function busName = createBusIfNeeded(rosMsgType, model)
            validateattributes(rosMsgType, {'char'}, {'nonempty'});
            validateattributes(model, {'char'}, {});

            ros.slros.internal.bus.Util.loadROSLibraryDictionary();
            [busExists, busName] = ros.slros.internal.bus.Util.checkForBus(rosMsgType, model);
            if busExists
                return;
            end
            [emptyRosMsg, msgInfo] = ros.slroscpp.internal.bus.Util.newMessageFromSimulinkMsgType(rosMsgType);
            emptyRosMsg = ros.slros.internal.bus.getBusStructForROSMsg(emptyRosMsg, msgInfo,'ros');
            ros.slroscpp.internal.bus.createBusDefnInGlobalScope(emptyRosMsg, model);
        end

        function ret = createMATLABStructFromMsgType(emptyMsg, msgInfo)
            ret = ros.slros.internal.bus.getBusStructForROSMsg(emptyMsg, msgInfo,'ros');
        end

        function allMsgsMap = getAllMessageInfoMapForModel(model)
            allMsgsMap = ros.slros.internal.bus...
                .getAllMessageInfoMapForModel(model,...
                                              'BusUtilityObject',ros.slroscpp.internal.bus.Util);
        end

        % Following methods are used in
        % ros.slroscpp.internal.sim.ROSMsgToBusStructConverter and
        % ros.slroscpp.internal.sim.BusStructToROSMsgConverter classes to
        % for ROS1 specific bus and message conversions
        function busstruct = convertROSTimeStruct(busstruct, msgType)
            if strcmp(msgType, 'ros_time/Time') || strcmp(msgType,'ros_time/Duration')
                busstruct.Sec = double(busstruct.Sec);
                busstruct.Nsec = double(busstruct.Nsec);
            end
        end

        function tempStruct = getEmptyStringStruct(value)
            tempStruct = struct('Data', value);
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
            if isStringArray
                % string array (string[]) that is mapped to a std_msgs/String[]
                rosMsgInstance = rosmessage('std_msgs/String', 'DataFormat', 'struct');
            else
                % regular nested bus
                if isempty(rosmsg.(propertyName))
                    % variable-length array
                    rosMsgInstance = ros.slroscpp.internal.bus.Util.newMessageFromSimulinkMsgType(elemInfo.MsgType);
                else
                    rosMsgInstance = rosmsg.(propertyName)(1);
                end
            end
        end

        function ret = convertBusstructToROSMsg(msg, msgInfo, emptyMsg)
        % CONVERTBUSSTRUCTTOROSMSG Create a ROS message structure
        % given a Simulink Bus structure representation
            getCachedMsgFcn = 'ros.slroscpp.internal.bus.Util.newMessageFromSimulinkMsgType';
            ret = ros.slros.internal.bus.convertBetweenROSMsgAndSLBus(msg, ...
                                                              msgInfo, emptyMsg, 'BUS2MSG', getCachedMsgFcn);
        end

        function ret = convertROSMsgToBusstruct(msg, msgInfo, emptyMsg)
        % CONVERTROSMSGTOBUSSTRUCT Create a Simulink Bus MATLAB
        % structure for use with bus-creation from a ROS 2 message (by
        % removing all the constant fields)
            getCachedMsgFcn = 'ros.slroscpp.internal.bus.Util.newMessageFromSimulinkMsgType';
            ret = ros.slros.internal.bus.convertBetweenROSMsgAndSLBus(msg, ...
                                                              msgInfo, emptyMsg, 'MSG2BUS', getCachedMsgFcn);
        end

        function ret = extractStringData(val, varargin)
        % EXTRACTSTRINGDATA Extract the 'Data' field from the variable
        % length nested string array.
            ret = {val.Data};
        end

        function ret = toStruct(msg, msgType)
        % TOSTRUCT Return equivalent structure of message without
        % constant fields get message info. This is equivalent to using
        % the "toStruct" method on ROS 1 message object

            [emptyMsg, msgInfo] = ros.slroscpp.internal.bus.Util.newMessageFromSimulinkMsgType(msgType);
            ret = ros.slroscpp.internal.bus.Util.convertROSMsgToBusstruct(msg, msgInfo, emptyMsg);
        end

        function [bus,busName] = getBusObjectFromMsgType(rosMsgType, model)
            busName = ros.slroscpp.internal.bus.Util.createBusIfNeeded(rosMsgType, model);
            bus = ros.slros.internal.evalinGlobalScope(bdroot(model), busName);
        end

        function [rosMsg, msgInfo] = newMessageFromSimulinkMsgType(rosMsgType)
        %newMessageFromSimulinkMsgType Create a new ROS message from message type
        %   Please note that ROSMSGTYPE could refer to a standard ROS
        %   message type, or to a pseudo-message type used only in
        %   Simulink, for example time and duration types.

            persistent msgCacheMap
            if isempty(msgCacheMap)
                msgCacheMap = containers.Map;
            end
            if isKey(msgCacheMap, rosMsgType)
                cachedInfo = msgCacheMap(rosMsgType);
                rosMsg = cachedInfo.Message;
                msgInfo = cachedInfo.Info;
            else
                if strcmp(rosMsgType, ros.slros.internal.bus.Util.TimeMessageType) || ...
                        strcmp(rosMsgType, 'ros/Time')
                    [rosMsg,msgInfo] = ros.internal.getEmptyMessage('ros/Time','ros');
                    msgInfo.MessageType = ros.slros.internal.bus.Util.TimeMessageType;
                elseif strcmp(rosMsgType, ros.slros.internal.bus.Util.DurationMessageType)|| ...
                        strcmp(rosMsgType, 'ros/Duration')
                    [rosMsg,msgInfo] = ros.internal.getEmptyMessage('ros/Duration','ros');
                    msgInfo.MessageType = ros.slros.internal.bus.Util.DurationMessageType;
                else
                    info = ros.internal.ros.getMessageInfo(rosMsgType);
                    try
                        [rosMsg, msgInfo] = eval(info.msgStructGen);
                    catch ex
                        if strcmp(ex.identifier, 'MATLAB:undefinedVarOrClass') || ...
                                isempty(info.pkgName) || isempty(info.msgName)
                            error(message('ros:utilities:message:MessageNotFoundError', rosMsgType, 'rosmsg list'))
                        else
                            newEx = MException(message('ros:utilities:message:CreateError', rosMsgType));
                            throw(newEx.addCause(ex));
                        end
                    end
                end
                msgCacheMap(rosMsgType) = struct('Message',rosMsg, 'Info', msgInfo);
            end
        end

        function ret = fromStruct(msgType, busstruct)
        % FROMSTRUCT Create an empty ROS 2 message and fill it with the
        % input Simulink Bus structure. This is equivalent to creating
        % an empty message using ''rosmessage'' and calling
        % ''fromStruct'' method of the ROS 1 message object.

            [emptyMsg, msgInfo] = ros.slroscpp.internal.bus.Util.newMessageFromSimulinkMsgType(msgType);
            ret = ros.slroscpp.internal.bus.Util.convertBusstructToROSMsg(busstruct, msgInfo, emptyMsg);
        end

        function ret = isStaticArray(msginfo)
            ret = ros.internal.ros2.MessageUtil.isStaticArray(msginfo);
        end

        function ret = isBoundedArray(~)
            ret = false;
        end
        
        function isFixed = isFixedSizeArray(msgType,propName,msginfo)
            isFixed = isfield(msginfo, 'MaxLen') && isfield(msginfo, 'MinLen') && ...
                isnumeric(msginfo.MaxLen) && (msginfo.MaxLen == msginfo.MinLen) && ...
                ((msginfo.MaxLen > 1) || ros.slros.internal.ROSUtil.isFixedSizeArray(msgType,propName));
        end

        % Check if all fields are constants
        function allConstants = isMessageAllConstants(msgType)
            persistent msgFieldsConstantMap__
            if isempty(msgFieldsConstantMap__)
                msgFieldsConstantMap__ = containers.Map;
            end
            if isKey(msgFieldsConstantMap__,msgType)
                allConstants = msgFieldsConstantMap__(msgType);
            else
                [msg, msgInfo] = ros.slroscpp.internal.bus.Util.newMessageFromSimulinkMsgType(msgType);
                allFields = fieldnames(msg);
                allFields(strcmp(allFields,'MessageType')) = [];
                allConstants = all(cellfun(@(f)msgInfo.(f).constant, allFields));
                msgFieldsConstantMap__(msgType) = allConstants;
            end
        end

        % Get ROSPropertyLists
        function rosPropList = getROSPropertyList(msgType,propList)
        % getROSPropertyList Convert the ROS MATLAB message structure
        % field names (MixedCase format) to ROS C++ property names
        % (lower-case underscore or snake_case format)
            persistent msgClassFieldMap__
            if isempty(msgClassFieldMap__)
                msgClassFieldMap__ = containers.Map;
            end
            if isKey(msgClassFieldMap__, msgType)
                rosPropList = msgClassFieldMap__(msgType);
            else
                [~,msgInfo]= ros.slroscpp.internal.bus.Util.newMessageFromSimulinkMsgType(msgType);
                %extract snake_case fields for C++ ROS message
                msgFlds = msgInfo.MatPath(~contains(msgInfo.MatPath,'.'));
                rosPropList = cellfun(@(x)ros.slroscpp.internal.bus.Util.getROSPropertyFromClassFields(x, msgFlds),...
                                      propList,'UniformOutput',false);
                msgClassFieldMap__(msgType) = rosPropList;
            end
        end
        
        function out = getROSPropertyFromClassFields(val, msgFlds)
            % For ROS C++ (structure) messages bus elements are CamelCase
            % and ROS Properties are in snake_case.

            ret = ros.internal.utilities.convertCamelcaseToLowercaseUnderscore(val);
            ret = regexprep(ret,'(_)$','');
            fldIdx = strcmpi(strrep(ret,'_',''),strrep(msgFlds,'_',''));
            if ismember(ret,msgFlds)
                out = ret;
            elseif any(fldIdx)
                % reserved field names
                out = msgFlds{fldIdx};
            else
                % Meta-fields like SL_Info and SL_DummyData that are not
                % used for creating message info map
                out = val;
            end
        end        

    end

end
