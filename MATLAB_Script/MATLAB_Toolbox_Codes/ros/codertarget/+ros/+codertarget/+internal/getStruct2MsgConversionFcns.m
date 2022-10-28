function [struct2RosMsgFcnMap, rosMsg2StructFcnMap] = getStruct2MsgConversionFcns(emptyROSMsg, varargin)
%

%   Copyright 2020-2021 The MathWorks, Inc.

    p = inputParser;
    addParameter(p, 'BusUtilityObject', ros.slroscpp.internal.bus.Util, @(x)isa(x,'ros.slros.internal.bus.Util'));
    addParameter(p, 'CodeGenUtilityObject', ros.slroscpp.internal.cgen.Util, @(x)isa(x,'ros.slros.internal.cgen.Util'));
    parse(p, varargin{:});
    busutilObj = p.Results.BusUtilityObject;
    cgenUtilObj = p.Results.CodeGenUtilityObject;
    params.ConvertFromStructFcnStr = 'struct2msg';
    params.ConvertToStructFcnStr = 'msg2struct';
    params.Indent = 2;
    struct2RosMsgFcnMap = containers.Map;
    rosMsg2StructFcnMap = containers.Map;
    generateConversionFcns('bus2cpp', emptyROSMsg, params, struct2RosMsgFcnMap, cgenUtilObj, busutilObj);
    generateConversionFcns('cpp2bus', emptyROSMsg, params, rosMsg2StructFcnMap, cgenUtilObj, busutilObj);
    % There should be a 1-1 relation between
    % bus2cpp and cpp2bus conversion functions
    assert(all(strcmp(sort(keys(struct2RosMsgFcnMap)), sort(keys(rosMsg2StructFcnMap)))), 'Mismatch detected for %s', class(emptyROSMsg));
end

%%
function generateConversionFcns(direction, emptyROSMsg, params, fcnInfoMap, cgenUtilObj, busutilObj)
    convertFromBus2Cpp = strcmp(direction, 'bus2cpp');
    [cppRosClass, msgType] = cgenUtilObj.rosObjToCppClass(emptyROSMsg);
    if isa(busutilObj,'ros.slros2.internal.bus.Util') %TODO: move this to bus.Util method
        [~, msgInfo, msggenInfo] = ros.internal.getEmptyMessage(msgType,'ros2');
        rosVer = 'ros2';
    else
        inMsgType = strrep(msgType,'ros_time/Duration','ros/Duration');
        inMsgType = strrep(inMsgType,'ros_time/Time','ros/Time');
        [~, msgInfo, msggenInfo] = ros.internal.getEmptyMessage(inMsgType,'ros');
        rosVer = 'ros';
    end
    if fcnInfoMap.isKey(cppRosClass)
        % information already present
        return;
    end
    params.MsgType = msgType;

    msgName = [msggenInfo.pkgName '_' msggenInfo.msgName];
    hashString = ros.slros.internal.bus.Util.hashString(msgName);
    % MATLAB has 63 character limit on function names
    % This is to match the generated struct from
    % ros.codertarget.internal.getEmptyCodegenMsg
    maxCharLimit = 63;
    chopLen = length('Struct') + length(['_', hashString]);
    endIdx = maxCharLimit - chopLen;
    structName = [msgName 'Struct_T'];
    if maxCharLimit < (length(msgName) + length('Struct'))
        structName = [msgName(1:endIdx),'_',hashString,'Struct_T'];
    end

    propList = setdiff(fieldnames(emptyROSMsg), 'MessageType');
    % MATLAB and ROS properties are the same for ROS2
    rosPropList = busutilObj.getROSPropertyList(msgType, propList);
    bus2CppMsgDef = sprintf(cgenUtilObj.getBusToCppMsgDefString(),cppRosClass);
    cpp2BusMsgDef = sprintf(cgenUtilObj.getCppToBusMsgDefString(),cppRosClass);

    %     bus2CppMsgDef = sprintf('%s* msgPtr',cppRosClass);
    %     cpp2BusMsgDef = sprintf('%s const* msgPtr',cppRosClass);

    % -- Generate function signature
    fcnSignature = StringWriter;

    if convertFromBus2Cpp
        fcnSignature.add('void %s(%s, %s const* structPtr)', ...
                         params.ConvertFromStructFcnStr, bus2CppMsgDef, structName);
    else
        fcnSignature.add('void %s(%s* structPtr, %s)', ...
                         params.ConvertToStructFcnStr, structName, cpp2BusMsgDef);
    end

    % -- Generate function body
    fcnBody = StringWriter;
    fcnBody.addcr('{');
    fcnBody.Indent = params.Indent;
    upstreamDependencies = {};

    fcnBody.addcr('const std::string rosMessageType("%s");', params.MsgType);
    fcnBody.addcr;

    for idx=1:numel(propList)
        matlabPropName = propList{idx};
        propinfo.MATLABPropName = matlabPropName;
        propinfo.BusPropName = matlabPropName;
        propinfo.CppPropName = rosPropList{idx};  % property name used in C++ object
        data = emptyROSMsg.(propinfo.MATLABPropName);
        currMsgInfo = msgInfo.(matlabPropName);
        if isfield(currMsgInfo, 'constant') && currMsgInfo.constant
            % Skip constant fields
            continue;
        end
        if isempty(data) && isfield(currMsgInfo,'MessageType')
            data = ros.internal.getEmptyMessage(currMsgInfo.MessageType,'ros');
        end

        elemInfo  = getMessageMetaInfo(data, currMsgInfo);
        elemInfo.isFixedLenArray = busutilObj.isFixedSizeArray(msgType, matlabPropName, currMsgInfo);
        elemInfo.isBoundedArray = busutilObj.isBoundedArray(currMsgInfo);
        propinfo.IsLogicalOrNumericType = elemInfo.isBuiltinType;
        propinfo.IsStringType = elemInfo.isStringType;
        propinfo.IsStringArray = elemInfo.isStringType && (elemInfo.isVarsizeArray || elemInfo.isFixedLenArray || elemInfo.isBoundedArray);
        propinfo.IsVarsizeArray = elemInfo.isVarsizeArray || elemInfo.isBoundedArray;
        propinfo.IsFixedLenArray = elemInfo.isFixedLenArray;
        propinfo.IsCharType = elemInfo.isCharType;
        propinfo.IsRosEntity = elemInfo.isRosMessage;
        propinfo.ArrayInfoPropName = '';
        propinfo.WarnOnTruncate = true;
        if (isa(data, 'int64') || isa(data, 'uint64'))
            propinfo.Int64Type = class(data);
        else
            propinfo.Int64Type = '';
        end

        % Determine if the value is a scalar type or not. All properties that
        % are neither a fixed-length array nor a variable-length array are
        % scalar.
        propinfo.IsScalar = ~(propinfo.IsFixedLenArray  || propinfo.IsVarsizeArray);

        if propinfo.IsRosEntity
            % Get an instance of the ROS entity so that it can be used for
            % recursion. If data is an empty-variable length array,
            % getDataInstance will create an instance.
            %
            % Note: a string array is specifically not a ROS entity (i.e., a
            % nested ROS message). In Simulink it is mapped to an array of
            % std_msgs/String, but in ROS C++ there is no corresponding
            % std_msgs/String object.
            upstreamDependencies{end+1} = getDataInstance(data); %#ok<AGROW>
        end

        if  propinfo.IsStringArray
            % string[] is treated as a variable-length array of std_msgs/String
            % In the MATLAB ROS message, string[] is not a ROS message.
            % string[] maps to a variable-size MATLAB cell string under the
            % message structure. This is not supported by MATLAB Coder.
            % Structures can't have members of type cell. Hence we simply
            % skip this field and not support setting / getting message
            % fields of type string[].
            fcnBody.addcr(['//WARNING: Do not use %s field. %s is a string[] ',...
                           'that is not supported for code generation.'],propinfo.BusPropName);
            %assert(~propinfo.IsRosEntity);
            %copyArray(convertFromBus2Cpp, propinfo, params, fcnBody, 'StringArray');

        elseif propinfo.IsStringType
            % string (treated as variable-length array of uint8)
            copyArray(convertFromBus2Cpp, propinfo, params, fcnBody, 'PrimitiveArray',rosVer);

        elseif propinfo.IsScalar
            % Scalar property: numeric, logical or nested message
            simpleAssignment(convertFromBus2Cpp, propinfo, params, fcnBody, cgenUtilObj);

        elseif propinfo.IsLogicalOrNumericType || propinfo.IsCharType
            % Array of primitive type (fixed, bounded or variable-length)
            copyArray(convertFromBus2Cpp, propinfo, params, fcnBody, 'PrimitiveArray',rosVer);

        else
            % Array of nested messages (fixed, bounded or variable-length)
            assert(propinfo.IsRosEntity);
            copyArray(convertFromBus2Cpp, propinfo, params, fcnBody, 'NestedArray',rosVer);
        end
    end
    fcnBody.Indent = fcnBody.Indent - params.Indent;
    fcnBody.addcr('}');

    fcnInfo.FcnSignature = fcnSignature;
    fcnInfo.FcnBody = fcnBody;
    fcnInfo.CppRosClass = cppRosClass;
    fcnInfo.SLBusName = structName;

    fcnInfoMap(cppRosClass) = fcnInfo;

    for i=1:numel(upstreamDependencies)
        generateConversionFcns(direction, upstreamDependencies{i}, params, fcnInfoMap, cgenUtilObj, busutilObj);
    end
end


%%
function datainstance = getDataInstance(data)
% Return need an instance of a ROS message. If data is empty (indicating a
% empty variable-length array), construct an instance of the message.
    if isempty(data)
        datainstance = feval(class(data));
    else
        datainstance = data(1);
    end
end

%%
function [from,to] = getFromAndTo(convertFromBus2Cpp, propinfo, params)

    to.assignmentCastStr = '';
    if convertFromBus2Cpp
        from.PtrStr = 'structPtr';
        from.PropName = propinfo.BusPropName;
        to.PtrStr = 'msgPtr';
        to.PropName = propinfo.CppPropName;
        to.conversionFcnStr = params.ConvertFromStructFcnStr;
        if ~isempty(propinfo.Int64Type)
            if strcmpi(propinfo.Int64Type, 'int64')
                to.assignmentCastStr = '(int64_t)';
            else
                to.assignmentCastStr = '(uint64_t)';
            end
        end
    else % cpp2bus
        from.PtrStr = 'msgPtr';
        from.PropName = propinfo.CppPropName;
        to.PtrStr = 'structPtr';
        to.PropName = propinfo.BusPropName;
        to.conversionFcnStr = params.ConvertToStructFcnStr;
        if propinfo.IsStringType
            to.assignmentCastStr = '(uint8_T)';
        end
        if ~isempty(propinfo.Int64Type)
            to.assignmentCastStr = '(real_T)'; % real_T is 64-bit float
        end
    end
end

%%
function simpleAssignment(convertFromBus2Cpp, propinfo, params, buf, cgenUtilObj)
    [from,to] = getFromAndTo(convertFromBus2Cpp, propinfo, params);
    assignmentString = cgenUtilObj.getSimpleAssignmentString(convertFromBus2Cpp, propinfo.IsRosEntity);
    if propinfo.IsLogicalOrNumericType || propinfo.IsCharType
        buf.addcr(assignmentString, to.PtrStr, to.PropName, to.assignmentCastStr, from.PtrStr, from.PropName);
    elseif propinfo.IsRosEntity
        % nested message
        buf.addcr(assignmentString, to.conversionFcnStr, to.PtrStr, to.PropName, from.PtrStr, from.PropName);
    else
        assert(false);
    end
end

%%
% Called for either fixed-length arrays (length > 1), or variable-length arrays
function copyArray(convertFromStruct, propinfo, params, buf, fcnNameSuffix,rosVer)
    v = validatestring(rosVer, {'ros', 'ros2'});
    validatestring(fcnNameSuffix, {'PrimitiveArray', 'StringArray', 'NestedArray'});

    % This function will generate one of the following functions (these are
    % explicitly listed here for find-ability in a code search):
    %
    % For PrimitiveArray
    %   convertToStructPrimitiveArray
    %   convertFromStructPrimitiveArray
    %
    % For StringArray
    %   convertToStructStringArray
    %   convertFromStructStringArray
    %
    % For NestedArray
    %   convertToStructNestedArray
    %   convertFromStructNestedArray

    [from,to] = getFromAndTo(convertFromStruct, propinfo, params);

    copyArrStringMap = containers.Map;
    copyArrStringMap('ros') = struct('FromStruct','convertFromStruct%s(%s->%s, %s->%s);',...
                                     'ToStruct','convertToStruct%s(%s->%s, %s->%s);');
    copyArrStringMap('ros2') = struct('FromStruct','convertFromStruct%s(%s.%s, %s->%s);',...
                                      'ToStruct','convertToStruct%s(%s->%s, %s.%s);');
    copyArrStruct = copyArrStringMap(v);
    if convertFromStruct
        buf.addcr(copyArrStruct.FromStruct, fcnNameSuffix, to.PtrStr, to.PropName, ...
                  from.PtrStr, from.PropName);
    else
        buf.addcr(copyArrStruct.ToStruct, fcnNameSuffix, to.PtrStr, to.PropName, ...
                  from.PtrStr, from.PropName);
    end
end

function ret  = getMessageMetaInfo(msg, msginfo)
    isROSTimeMsgFcn = @(x)( isequal(x,'ros/Time') || isequal(x,ros.slros.internal.bus.Util.TimeMessageType));
    isROSDurationMsgFcn = @(x)( isequal(x,'ros/Duration') || isequal(x,ros.slros.internal.bus.Util.DurationMessageType));
    ret = struct;
    ret.isROSTimeEntity = isfield(msginfo,'MessageType') && ...
        isROSTimeMsgFcn(msginfo.MessageType);
    ret.isROSDurationEntity = isfield(msginfo,'MessageType') && ...
        isROSDurationMsgFcn(msginfo.MessageType);
    ret.isMaxLenNan    = isfield(msginfo, 'MaxLen') && isnan(msginfo.MaxLen);

    ret.isBoundedArray = ros.slroscpp.internal.bus.Util.isBoundedArray(msginfo);

    ret.isBuiltinType = isa(msg,'logical') || isa(msg,'numeric');

    ret.isStringType = isfield(msginfo, 'MLdataType') && isequal(msginfo.MLdataType, 'string');

    ret.isCharType = isfield(msginfo, 'MLdataType') && isequal(msginfo.MLdataType, 'char');

    ret.isRosMessage = isfield(msginfo, 'MessageType') && isfield(msginfo, 'MLdataType') && isequal(msginfo.MLdataType, 'struct');

    ret.isVarsizeArray = ret.isMaxLenNan || ret.isBoundedArray;
end

% LocalWords:  rosmessage slros cgen roscpp
