function requiredBuses = getBusDefnForROSMsg(emptyRosMsg, model, varargin)
%This function is for internal use only. It may be removed in the future.

%getBusDefnForROSMsg - Create Simulink bus object corresponding to a ROS message
%
%    getBusDefnForROSMsg(MSG) returns an array of Simulink.Bus objects
%    corresponding to ROS message MSG and any nested messages inside it.
%    MSG should be an empty message (i.e., created using ROSMESSAGE without
%    any subsequent assignments), since the only way to determine if a
%    property is a variable-length array is to check if its value is [].
%
%    Note:
%    * This function does not create the bus objects in the global scope.
%      Consequently, this function does not define the "name" of the bus
%      (i.e., the name of the bus in the global scope).
%
%    * If the ROS message has variable-size array properties, these are
%      converted to fixed-length arrays (with length of 1), and the
%      associated metadata element is added to the bus object.
%
%    * This function returns a bus object (which is a bus definition), &
%      not an instance of a bus (a bus signal).

%   Copyright 2014-2020 The MathWorks, Inc.

% Good tests for this function:
%
%  getBusDefnForROSMsg( rosmessage('sensor_msgs/JointState') , '')
%
%  getBusDefnForROSMsg( ros2message('sensor_msgs/JointState') , '', @ros.slros.internal.bus.Util.processBus, 'ros')
%
%  getBusDefnForROSMsg( ros2message('sensor_msgs/JointState') , '', @ros.slros.internal.bus.Util.processBus, 'ros2')

    map = containers.Map;
    if nargin < 3
        processBus(emptyRosMsg, model, map);
    else
        validateattributes(varargin{1}, {'function_handle'}, {'nonempty'},'getBusDefnForROSMsg','processBusFcn', 3);
        feval(varargin{1}, emptyRosMsg, model, map);
    end
    requiredBusesCellArray = values(map);
    requiredBuses = [requiredBusesCellArray{:}];

end

%%
function processBus(emptyRosMsg, model, map)
    if isempty(emptyRosMsg)
        % nothing to do
        return;
    end

    % should not be making any recursive calls with arrays of messages
    assert(numel(emptyRosMsg)==1);

    if ros.slros.internal.ROSUtil.isROSTimeEntityObj(emptyRosMsg)
        msgType = ros.slros.internal.bus.Util.rosTimeToQuasiMsgType(class(emptyRosMsg));
    else
        assert(all(isprop(emptyRosMsg, 'MessageType')));
        msgType = emptyRosMsg.MessageType;
    end

    if isKey(map, msgType)
        % nothing to do
        return;
    end

    propertyList = fieldnames(emptyRosMsg);

    % During a simulation, we will use msg.toStruct() to convert the ROS
    % message to a Simulink bus struct. Consequently, it is important for the
    % **order of elements** in the bus struct to be as close as possible to the
    % order of fields returned by msg.toStruct(). Hence, call toStruct() here
    % to get a "canonical order", and add elements to the bus definition in
    % that order.

    isRosTimeEntity = ros.slros.internal.ROSUtil.isROSTimeEntityObj(emptyRosMsg);
    if isRosTimeEntity
        % no toStruct() method defined for time entities
        canonicalOrder = propertyList;
    else
        canonicalOrder = fieldnames(toStruct(emptyRosMsg));
        % The toStruct() leaves out some fields. Splice these back in (though
        % these will likely just be MessageType and Constant properties, which
        % will be discarded later on anyway).
        canonicalOrder = [setdiff(propertyList, canonicalOrder) ; canonicalOrder];
    end

    % Use metaclass, as we need to distinguish Constant properties below
    rosmsgMetaClass = metaclass(emptyRosMsg);
    rosmsgAllProps = {rosmsgMetaClass.PropertyList.Name};

    busElements = Simulink.BusElement.empty;

    if ros.slros.internal.ROSUtil.isStdEmptyMsgType(msgType) || ...
            (numel(canonicalOrder) == 1) && strcmpi(canonicalOrder{1}, 'MessageType')

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

        metaprop = rosmsgMetaClass.PropertyList(strcmp(propertyName, rosmsgAllProps));
        isConstant = metaprop.Constant;
        if isConstant
            % Constants are not yet supported
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
            % remember the original (unmangled) name
            elemInfo.ROSPropName = propertyName;
        end

        data = emptyRosMsg.(propertyName);

        isLogicalOrNumericType = isa(data,'logical') || isa(data,'numeric');
        isStringType = isa(data, 'char');
        isStringArrayType = isa(data, 'cell');
        isRosTimeEntity = ros.slros.internal.ROSUtil.isROSTimeEntityObj(data);
        % isROSMsgObj returns true even if data is zero-length array of expected type
        isRosMessage = ros.slros.internal.ROSUtil.isROSMsgObj(data);

        %------
        % Handle primitive and complex message properties

        if isStringArrayType
            % Variable or fixed-length array of strings.
            % Replace the 'data' cell array with an array of std_msgs/String
            numStrings = numel(data);
            tmpStringMsg = ros.slros.internal.ROSUtil.getStdStringObj();
            if numStrings == 0
                % variable-length array of strings
                data = tmpStringMsg.empty(0,1);
            else
                % fixed-length array of strings
                % Note
                % 1) We are only using the 'data' variable to set up datatypes;
                % the actual content of 'data' is not used or modified.
                % 2) We can't just assign to data(1:numStrings,1); that syntax
                % requires LHS and RHS to have matching types, whereas here we
                % are replacing a cell array with a new data type.
                data = tmpStringMsg;
                data(2:numStrings,1) = tmpStringMsg;
            end
            rosMessageType = tmpStringMsg.MessageType;
            dataTypeStr = ros.slros.internal.bus.Util.rosMsgTypeToDataTypeStr(rosMessageType, model);
            elem.DataType = dataTypeStr;
            elemInfo.MsgType = rosMessageType;
            elemInfo.PrimitiveROSType = 'string[]';
            processBus(tmpStringMsg, model, map); % recursive call

        elseif isLogicalOrNumericType
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

        elseif isStringType
            elem.DataType = 'uint8';
            elemInfo.PrimitiveROSType = 'string';  % remember that this uint8 array is a string

        elseif isRosTimeEntity
            datainstance = getDataInstance(data);
            [dataTypeStr, ~, timeMsgType] = ros.slros.internal.bus.Util.rosTimeToDataTypeStr(class(data), model);
            elem.DataType = dataTypeStr;
            elemInfo.MsgType = timeMsgType;
            processBus(datainstance, model, map); % recursive call

        elseif isRosMessage
            datainstance = getDataInstance(data);
            assert(isprop(datainstance, 'MessageType'));
            rosMessageType = datainstance.MessageType;
            dataTypeStr = ros.slros.internal.bus.Util.rosMsgTypeToDataTypeStr(rosMessageType, model);
            elem.DataType = dataTypeStr;
            elemInfo.MsgType = rosMessageType;
            processBus(datainstance, model, map); % recursive call

        else
            assert(false);
        end

        %------
        % Add metadata related to variable vs. fixed-length arrays

        isVarsizeArray = isempty(data) || isStringType;
        busElements = ros.slros.internal.bus.setBusDimensions(elemInfo, elem, busElements, data, isVarsizeArray);
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
