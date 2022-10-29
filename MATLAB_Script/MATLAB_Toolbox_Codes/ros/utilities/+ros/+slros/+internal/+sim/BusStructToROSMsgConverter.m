classdef BusStructToROSMsgConverter < handle
%This class is for internal use only. It may be removed in the future.

%  BusStructToROSMsgConverter(MSGTYPE, MODEL) creates an object that
%  converts a Simulink bus struct, corresponding to a ROS message type
%  MSGTYPE, to a MATLAB ROS message. This conversion uses the maximum
%  sizes of variable-length properties that is specific to MODEL.
%
%  Example:
%   msg2bus = ros.slros.internal.sim.ROSMsgToBusStructConverter('nav_msgs/Path');
%   bus2msg = ros.slros.internal.sim.BusStructToROSMsgConverter('nav_msgs/Path');
%   busstruct = msg2bus.convert(pathMsg);
%   msg2 = bus2msg.convert(busstruct); % convert back
%
% See also: sim.ROSMsgToBusStructConverter

%   Copyright 2014-2019 The MathWorks, Inc.

    properties
        ROSMessageType
        Model
        MsgInfoMap
        BusUtil
    end

    methods
        function obj = BusStructToROSMsgConverter(msgtype, model, varargin)
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
                'ModelName', model, ...
                'MapKeyType', 'msgtype', ...
                'Recurse', true), ...
                'BusUtilityObject', obj.BusUtil);
            assertInfoStructNames(obj.BusUtil);
        end

        function rosmsg = convert(obj, busstruct)
            busstruct = processStruct(busstruct, obj.MsgInfoMap, obj.ROSMessageType, obj.BusUtil);
            rosmsg = obj.BusUtil.fromStruct(obj.ROSMessageType, busstruct);
        end

    end

end

%%
%%
function assertInfoStructNames(busUtil)
% processStruct() assumes the variable-array info struct has a field
% name  named 'CurrentLength' (doesn't look up the fieldname at runtime).
% Verify that this assumption is valid.
    infostruct = busUtil.getArrayInfoStructMetadata();
    assert(strcmp(infostruct.CurLengthProp, 'CurrentLength'));
end

%%
function busstruct = processStruct(busstruct, map, msgtype, busUtil)

    info = map(msgtype);

    % For list of expected fields in info, see
    % createROSMsgInfoMap:processBus()

    % if ~isempty(info.StdEmptyMsgProp)
    %   Nothing to do. We silently ignore (throw away) the dummy property
    % end

    % Ensure variable-length array information is captured in the
    % metadata (_SL_Info) field
    for i=1:numel(info.VarLenArrays)
        prop = info.VarLenArrays{i};
        infoprop = info.VarLenArrayInfoProps{i};

        maxlen = info.VarLenMaxLen{i};
        actuallen = numel(busstruct.(prop));
        validlen = busstruct.(infoprop).CurrentLength;

        % Note:
        % 1) busstruct.(lengthprop).ReceivedLength will be discarded
        %    (not meaningful during bus -> ROS message conversion)
        %
        % 2) In practice, actuallen will be equal to validlen (there is no
        %    way in Simulink to extend the max length of an array during a
        %    simulation). However, this function receives an anonymous struct
        %    & can't make this assumption, so we handle the case where
        %    actuallen > validlen

        assert((validlen <= actuallen) && (actuallen <= maxlen));

        if validlen < actuallen
            busstruct.(prop)(validlen+1:actuallen) = []; % remove extra elements
        end
    end

    % Uint64 and Int64 values will be handled by fromStruct() for the ROS
    % Message (i.e., it will cast them appropriately, so no need for explicit
    % casts back to double)
    
    % convert uint8 to chars
    for i=1:numel(info.CharProps)
        prop = info.CharProps{i};
        busstruct.(prop) = busUtil.convertStringsFromBusToMsg(busstruct.(prop));
    end
    
    % convert uint8 to strings
    for i=1:numel(info.StringProps)
        prop = info.StringProps{i};
        busstruct.(prop) = busUtil.convertStringsFromBusToMsg(busstruct.(prop)); 
    end

    % no need to remove "CurrentLength" fields; they are ignored

    % Future optimization: we don't have to process all arrays, only those
    % with a variable-length array under them (and this is discoverable through
    % static analysis)

    % note that we've already trimmed the length of variable-length arrays, so
    % they have the correct length.
    for i=1:numel(info.NestedMsgs)
        prop = info.NestedMsgs{i};
        nestedMsgType = info.NestedMsgType{i};
        len = numel(busstruct.(prop));  % how about multi-dimensional arrays?

        substructs = struct.empty;
        for j=1:len
            tmpout = processStruct( ...
                busstruct.(prop)(j), ...
                map, ...
                nestedMsgType, ...
                busUtil);
            if isempty(substructs)
                substructs = tmpout;
            else
                substructs(j) = tmpout; % extend as row or column?
            end
        end

        busstruct.(prop) = substructs;
    end

    % map back to a cell array of strings
    for i=1:numel(info.StringArrayProps)
        prop = info.StringArrayProps{i};
        proplen = info.StringArrayLen{i};

        if proplen == 0
            % variable-length array
            infoprop = info.StringArrayInfoProps{i};
            validlen = busstruct.(infoprop).CurrentLength;
        else
            % fixed-length array
            validlen = proplen;
        end

        % Note that busstruct.(prop) has already been trimmed to the correct
        % length, and the busstruct.(prop).Data has already been converted to
        % a MATLAB string -- this is just the standard processing for any
        % array of std_msgs/String[].
    
        if (validlen == 0) || (isempty(busstruct.(prop)))
            busstruct.(prop) = {};
        else
            % for ros2 if the property is not in StringProps, implies it is
            % a nested string array in which case we need to create a cell
            % array
            busstruct.(prop) = busUtil.extractStringData(busstruct.(prop),~ismember(prop, info.StringProps));
        end
    end

    % If there are reserved names, make sure to have non-mangled
    % property names and remove the mangled property names.
    for i=1:numel(info.ReservedWordProp)
        busName = info.ReservedWordProp(i).BusName;
        rosName = info.ReservedWordProp(i).RosName;
        % Copy the content to the non-mangled property name, so it can be used
        % to reconstruct the ROS message object
        busstruct.(rosName) = busstruct.(busName);
        % Remove the mangled property name
        busstruct = rmfield(busstruct, busName);
    end

end
