classdef ROSMsgToBusStructConverter < handle
%This class is for internal use only. It may be removed in the future.

%  ROSMsgToBusStructConverter(MSGTYPE, MODEL) creates an object that
%  converts a MATLAB ROS message, with message type MSGTYPE, to a
%  corresponding Simulink bus struct. This conversion uses the maximum
%  sizes of variable-length properties that is specific to MODEL.
%
%  Example:
%   msg2bus = ros.slros.internal.sim.ROSMsgToBusStructConverter('nav_msgs/Path', modelName);
%   bus2msg = ros.slros.internal.sim.BusStructToROSMsgConverter('nav_msgs/Path', modelName);
%   busstruct = msg2bus.convert(pathMsg);
%   msg2 = bus2msg.convert(busstruct); % convert back
%
% See also: sim.BusStructToROSMsgConverter

%   Copyright 2014-2020 The MathWorks, Inc.

    properties
        ROSMessageType
        Model
        MsgInfoMap
        BusUtil
    end

    methods
        function obj = ROSMsgToBusStructConverter(msgtype, model, varargin)
            p = inputParser();
            addRequired(p, 'msgtype', @(x)ischar(x) && ~isempty(x));
            addRequired(p, 'model', @(x)ischar(x) && ~isempty(x));
            addParameter(p, 'BusUtilityObject', ros.slros.internal.bus.Util, @(x)isa(x,'ros.slros.internal.bus.Util'));
            parse(p, msgtype, model, varargin{:});
            obj.ROSMessageType = p.Results.msgtype;
            obj.Model = p.Results.model;
            obj.BusUtil = p.Results.BusUtilityObject;
            obj.MsgInfoMap = ros.slros.internal.sim.createROSMsgInfoMap(struct(...
                'MessageType', obj.ROSMessageType, ...
                'ModelName', obj.Model, ...
                'MapKeyType', 'msgtype', ...
                'Recurse', true), ...
                'BusUtilityObject', obj.BusUtil);
        end

        function busstruct = convert(obj, msg)
            rosstruct = obj.BusUtil.toStruct(msg, obj.ROSMessageType);
            busstruct = rosstruct;
            busstruct = processStruct(rosstruct, busstruct, obj.MsgInfoMap, obj.ROSMessageType, obj.Model, obj.BusUtil);
        end
    end
end


%%
function busstruct = processStruct(rosstruct, busstruct, map, msgtype, modelname, busUtil)

    info = map(msgtype);

    % For list of expected fields in info, see
    % createROSMsgInfoMap:processBus()

    % Deal with special case fields

    if isfield(busstruct, 'MessageType')
        busstruct = rmfield(busstruct, 'MessageType');
    end
    busstruct = busUtil.convertROSTimeStruct(busstruct, info.ROSMessageType);

    % If any of the properties are a reserved name, the name in the bus is
    % mangled. All the info fields (e.g., StringArrayProps, NestedMsgs)
    % use the possibly-mangled name in the bus. The strategy here is to first
    % ensure that
    %   a) busstruct has the mangled name (e.g.  Vector_), and
    %   b) rosstruct has the mangled name as well
    % That way, the subsequent info field manipulations will work with
    % rosstruct.Vector_ and copy it to busstruct.Vector_

    if numel(info.ReservedWordProp) > 0
        rosNames = info.ReservedWordInfo.RosNames; % original names
        busNames = info.ReservedWordInfo.BusNames; % mangled names
                                                   % rename the fields corresponding to the reserved words
        busstruct = cell2struct(struct2cell(rosstruct), info.ReservedWordInfo.AdjustedFieldNames, 1);
        % copy rosstruct.(rosNames) to rosstruct.(busNames) as well
        for j=1:length(busNames)
            rosstruct.(busNames{j}) = rosstruct.(rosNames{j});
        end
    end


    for i=1:numel(info.StringArrayProps)
        % Cell arrays of strings
        % Convert the entire cell array to the toStruct equivalent of std_msgs/String[]
        % For variable-length string arrays, the VarLenArray processing loop
        %    (below) will do any required clipping
        % For fixed-length string arrays, we assume that the incoming length of
        %    the string array matches the length of the Simulink bus array (i.e.,
        %    nothing extra to be done)
        prop = info.StringArrayProps{i};
        assert(iscell(rosstruct.(prop)));
        tmpStruct = busUtil.getEmptyStringStruct(rosstruct.(prop));
        tmpStruct = busUtil.toStruct(tmpStruct, 'std_msgs/String');
        rosstruct.(prop) = tmpStruct;
        busstruct.(prop) = tmpStruct;
    end

    if ~isempty(info.StdEmptyMsgProp)
        dummyprop = info.StdEmptyMsgProp{1};
        busstruct.(dummyprop) = false; % assign dummy value
    end

    % Generic processing for all variable-length arrays (primitive numeric
    % arrays, arrays of nested messages, string arrays). This mainly consists
    % of setting the appropriate SL_Info values.
    for i=1:numel(info.VarLenArrays)
        prop = info.VarLenArrays{i};
        infoprop = info.VarLenArrayInfoProps{i};
        maxlength = info.VarLenMaxLen{i};
        recvdlength = busUtil.getLength(rosstruct, prop, info);
        currentlength = min(maxlength, recvdlength);

        if (max(recvdlength) > maxlength) && ...
                info.VarLenTruncateAction{i} == ros.slros.internal.bus.VarLenArrayTruncationAction.EmitWarning
            diag = MSLDiagnostic([], ...
                                 message('ros:slros:busconvert:TruncatedArray', ...
                                         prop, msgtype, maxlength, max(recvdlength), maxlength, modelname));
            reportAsWarning(diag);
        end

        busstruct.(infoprop) = busUtil.getVarlenInfoStruct(currentlength, recvdlength);
    end


    % convert int64 and uint64 to doubles (note that these can be arrays)
    for i=1:numel(info.Int64Props)
        prop = info.Int64Props{i};
        busstruct.(prop) = double(rosstruct.(prop));  % note the transpose to column vec
    end

    for i=1:numel(info.CharProps)
        prop = info.CharProps{i};
        busstruct.(prop) = busUtil.convertStringsToUint8(rosstruct.(prop));  % note the transpose to column vec
    end
    
    % convert strings to uint8
    for i=1:numel(info.StringProps)
        prop = info.StringProps{i};
        busstruct.(prop) = busUtil.convertStringsToUint8(rosstruct.(prop));  % note the transpose to column vec
    end

    % Specific handling for Variable-length primitive arrays
    %
    % For actual fixed-length arrays (those that are defined fixed by ROS),
    % we can assume they will come in at the correct length. But for
    % var-length arrays of primitive types, we need to enforce that the
    % length exactly matches the max-length. Note: the SL_Info property is
    % already set in the generic processing for all variable-length arrays
    for i=1:numel(info.VarLenPrimitiveArrays)
        prop = info.VarLenPrimitiveArrays{i};
        maxlen = info.VarLenPrimitiveMaxLen{i};

        len = numel(busstruct.(prop));
        if len < maxlen
            % ros array too short; zero-pad to the max length of the bus array
            busstruct.(prop)(len+1:maxlen) = 0; % 0 will get typecast appropriately
        elseif len  > maxlen
            % ros array too long; crop the bus array
            busstruct.(prop)(maxlen+1:end) = [];
        else
            % nothing to do
        end
    end

    % Specific handling for Variable-length message arrays
    % This is exactly analogous to the handling of primitive arrays.
    for i=1:numel(info.VarLenMsgArrays)
        prop = info.VarLenMsgArrays{i};
        maxlen = info.VarLenMsgMaxLen{i};

        % Need to remove MessageType field from sub-structs
        % or else info.VarLenMsgEmpty will not match the structure
        if isfield(busstruct.(prop), 'MessageType')
            busstruct.(prop) = rmfield(busstruct.(prop), 'MessageType');
        end

        len = numel(busstruct.(prop));
        if len == 0
            % ros array has zero length. This is a special case --
            % busstruct.(prop) doesn't have the expected struct type, so it
            % can't just be extended to the maximum length. Solution: use the
            % VarLenMsgEmpty to set up the array of desired type & length
            emptystruct = info.VarLenMsgEmpty{i};
            emptystruct(2:maxlen) = emptystruct(1);
            busstruct.(prop) = emptystruct;
            rosstruct.(prop) = info.VarLenMsgEmpty{i};
        elseif len < maxlen
            % ros array too short; zero-pad to the max length of the bus array
            % Can't just set busstruct.(prop)(maxlen), since the intervening
            % structs would get all fields set to [] (empty double array)
            busstruct.(prop)(len+1:maxlen) = info.VarLenMsgEmpty{i};
        elseif len  > maxlen
            % ros array too long; crop the bus array
            busstruct.(prop)(maxlen+1:end) = [];
        else
            % nothing to do
        end
    end


    % Specific handling for nested messages (including scalar, fixed-length
    % arrays, and variable-length arrays). We need to recurse over each of the
    % nested message instances to ensure that their data is appropriately
    % fixed-up.
    %
    % Note --
    % 1) All the bus arrays in this message have been fixed-up already
    %   (padded/trimmed to the maximum length, corresponding SL_Info values have
    %    been set).
    %
    % 2) busstruct is a pure struct data structure (i.e., all the nested arrays
    %    are struct arrays). This means they cannot be fixed-up "in-place"
    %    (esp. because the fixing-up will involve adding, deleting, and
    %    reordering struct fields). We need to get the fixed-up struct and replace
    %    the old struct with it.
    %
    for i=1:numel(info.NestedMsgs)
        prop = info.NestedMsgs{i};
        nestedMsgType = info.NestedMsgType{i};

        % Note: the following logic works handles scalar nested messages as well.
        % Also, we don't account for multi-dimensional arrays, since ROS
        % arrays are 1-d by definition.
        roslen = numel(rosstruct.(prop));
        buslen = numel(busstruct.(prop));

        % Can't initialize sub-structs with busstruct.(prop). The type of the
        % struct may not match type of fixed-up struct (e.g., due to added or
        % removed fields). We need to grow substruct with the correct fixed-up
        % struct type
        substructs = struct.empty;

        validLength = max(min(buslen,roslen),1);
        for j=1:validLength
            tmpout = processStruct( ...
                rosstruct.(prop)(j), ...
                busstruct.(prop)(j), ...
                map, ...
                nestedMsgType, ...
                modelname, ...
                busUtil);
            if isempty(substructs)
                substructs = tmpout;
            else
                substructs(j) = tmpout;
            end
        end

        if validLength < buslen
            % Replicate the last element of the ROS array in all the padding
            % elements (the bus array structs need to be fixed-up even if they
            % are all "null").
            substructs(validLength+1:buslen) = tmpout;
        end

        busstruct.(prop) = substructs;
    end
    busstruct = busUtil.permuteBusElements(busstruct, info.propertyReorderInfo);
end
