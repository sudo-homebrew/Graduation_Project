function defineSimulinkBus(emptyRosMsg, model, map)
%This function is for internal use only. It may be removed in the future.

%   Copyright 2019-2020 The MathWorks, Inc.

% DEFINESIMULINKBUS Defines Simulink.Bus objects by parsing the EMPTYROSMSG
% input recursively and appends it to the containers.Map input object, MAP.

    if isempty(emptyRosMsg)
        % nothing to do
        return;
    end
    % should not be making any recursive calls with arrays of messages
    assert(numel(emptyRosMsg)==1);
    assert(all(isfield(emptyRosMsg, 'MessageType')));
    msgType = emptyRosMsg.MessageType;
    [~, rosMsgMetaData, ~] = ros.internal.getEmptyMessage(msgType, 'ros2');
    if isKey(map, msgType)
        % nothing to do
        return;
    end

    allFields = fieldnames(emptyRosMsg);

    % During a simulation, we will use msg.toStruct() to convert the ROS
    % message to a Simulink bus struct. Consequently, it is important for the
    % **order of elements** in the bus struct to be as close as possible to the
    % order of fields returned by msg.toStruct(). Hence, call toStruct() here
    % to get a "canonical order", and add elements to the bus definition in
    % that order.

    canonicalOrder = allFields;

    busElements = Simulink.BusElement.empty;

    if ros.slros.internal.ROSUtil.isStdEmptyMsgType(msgType) || ...
            (numel(canonicalOrder) == 1) && strcmpi(canonicalOrder{1}, 'MessageType') || ...
            ros.slros2.internal.bus.Util.isMessageAllConstants(msgType)
        % The only field in an empty message should be 'MessageType'
        % Add in the dummy field (the loop below over canonicalOrder is effectively a no-op)
        elem = ros.slros.internal.bus.Util.getDummyElementForStdEmptyMsg;
        busElements(end+1) = elem;
    end

    nameChecker = ros.slros.internal.bus.ReservedNamesChecker.getInstance;

    for i =1:length(canonicalOrder)
        propertyName = canonicalOrder{i};

        if strcmp(propertyName,'MessageType')
            continue;
        end

        [elemName,isReservedName] = nameChecker.mangleNameIfNeeded(propertyName);

        elem = Simulink.BusElement;
        elem.Name = elemName;
        elem.Dimensions = 1;
        elem.SampleTime = -1;
        elem.Complexity = 'real';
        elem.SamplingMode = 'Sample based';
        elem.Min = [];
        elem.Max = [];
        elem.DocUnits = '';
        elem.Description = '';
        elemInfo = ros.slros.internal.bus.BusItemInfo;

        if isReservedName
            % remember the original (non-mangled) name
            elemInfo.ROSPropName = propertyName;
        end

        data = emptyRosMsg.(propertyName);
        msginfo = rosMsgMetaData.(propertyName);

        isConstant = isfield(msginfo, 'constant') &&  msginfo.constant;
        if isConstant
            % Constants are not yet supported
            continue;
        end

        isMaxLenNan    = isfield(msginfo, 'MaxLen') && isnan(msginfo.MaxLen);

        isBoundedArray = ros.slros2.internal.bus.Util.isBoundedArray(msginfo);

        isBuiltinType = isa(data,'logical') || isa(data,'numeric');

        isStringType = isfield(msginfo, 'MLdataType') && isequal(msginfo.MLdataType, 'string');

        isCharType = isfield(msginfo, 'MLdataType') && isequal(msginfo.MLdataType, 'char');

        isRosMessage = isfield(msginfo, 'MessageType') && isfield(msginfo, 'MLdataType') && isequal(msginfo.MLdataType, 'struct');

        isRosTimeEntity = isRosMessage && isequal(msginfo.MessageType, 'builtin_interfaces/Time');

        isVarsizeArray = isMaxLenNan || isBoundedArray;

        isStaticArray = ros.slros2.internal.bus.Util.isStaticArray(msginfo);

        isVarStringArray = false;


        %------
        % Handle primitive and complex message properties

        if isStringType
            if isVarsizeArray || isStaticArray
                numStrings = numel(data);
                dummyStringMsg = ros.slros2.internal.bus.Util.newMessageFromSimulinkMsgType('std_msgs/String');
                dummyStringMsg.MessageType = 'std_msgs/String';
                if numStrings == 0
                    % variable-length array of strings
                    data = repmat(dummyStringMsg,1,0);
                else
                    % fixed-length array of strings
                    % Note
                    % 1) We are only using the 'data' variable to set up datatypes;
                    % the actual content of 'data' is not used or modified.
                    % 2) We can't just assign to data(1:numStrings,1); that syntax
                    % requires LHS and RHS to have matching types, whereas here we
                    % are replacing a cell array with a new data type.
                    data = repmat(dummyStringMsg, numStrings,1);
                end
                dataTypeStr = ros.slros2.internal.bus.Util.rosMsgTypeToDataTypeStr('std_msgs/String');
                elem.DataType = dataTypeStr;
                elemInfo.MsgType =  'std_msgs/String';
                elemInfo.PrimitiveROSType = 'string[]';
                ros.slros.internal.bus.defineSimulinkBus(dummyStringMsg, model, map); % recursive call
                isVarStringArray = isVarsizeArray;
            else
                elem.DataType = 'uint8';
                elemInfo.PrimitiveROSType = 'string';
                isVarStringArray = true;
            end
        elseif isBuiltinType
            if isa(data,'logical')
                elem.DataType = 'boolean';
            elseif isa(data, 'int64') || isa(data, 'uint64')
                elem.DataType = 'double';
                elemInfo.Int64Type = class(data);
                robotics.internal.warningNoBacktrace(...
                    message('ros:slros:busconvert:Int64NotSupported', ...
                            propertyName, msgType, class(data), 'double'));
            else
                elem.DataType = class(data);
            end
        elseif isCharType
            elem.DataType = 'uint8';
            elemInfo.PrimitiveROSType = 'char';
        elseif (isRosMessage || isRosTimeEntity)
            datainstance = getDataInstance(data);
            assert(isfield(datainstance, 'MessageType'));
            rosMessageType = datainstance.MessageType;
            dataTypeStr = ros.slros2.internal.bus.Util.rosMsgTypeToDataTypeStr(rosMessageType);
            elem.DataType = dataTypeStr;
            elemInfo.MsgType = rosMessageType;
            ros.slros.internal.bus.defineSimulinkBus(datainstance, model, map); % recursive call
        else
            assert(false);
        end
        %------
        % Add metadata related to variable vs. fixed-length arrays
        isVarLen = isVarsizeArray || isVarStringArray;
        busElements = ros.slros.internal.bus.setBusDimensions(elemInfo, elem, busElements, data, isVarLen);
    end
    busObj = ros.slros.internal.bus.getBusObject(busElements, msgType);
    map(msgType) = struct('MessageType', msgType, 'Bus', busObj); %#ok<NASGU>
end

%%
function datainstance = getDataInstance(data)
    if isempty(data)
        datainstance = feval(class(data));
    else
        datainstance = data(1);
    end
end
