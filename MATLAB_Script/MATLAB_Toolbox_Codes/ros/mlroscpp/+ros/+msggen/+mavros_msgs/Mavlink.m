
classdef Mavlink < ros.Message
    %Mavlink MATLAB implementation of mavros_msgs/Mavlink
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'mavros_msgs/Mavlink' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '41093e1fd0f3eea1da2aa33a177e5ba6' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'FramingStatus' 'Magic' 'Len' 'IncompatFlags' 'CompatFlags' 'Seq' 'Sysid' 'Compid' 'Msgid' 'Checksum' 'Payload64' 'Signature' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'framing_status' 'magic' 'len' 'incompat_flags' 'compat_flags' 'seq' 'sysid' 'compid' 'msgid' 'checksum' 'payload64' 'signature' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        FRAMINGOK = uint8(1)
        FRAMINGBADCRC = uint8(2)
        FRAMINGBADSIGNATURE = uint8(3)
        MAVLINKV10 = uint8(254)
        MAVLINKV20 = uint8(253)
    end
    properties
        Header
        FramingStatus
        Magic
        Len
        IncompatFlags
        CompatFlags
        Seq
        Sysid
        Compid
        Msgid
        Checksum
        Payload64
        Signature
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'Mavlink', 'Header')
            obj.Header = val;
        end
        function set.FramingStatus(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Mavlink', 'FramingStatus');
            obj.FramingStatus = uint8(val);
        end
        function set.Magic(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Mavlink', 'Magic');
            obj.Magic = uint8(val);
        end
        function set.Len(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Mavlink', 'Len');
            obj.Len = uint8(val);
        end
        function set.IncompatFlags(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Mavlink', 'IncompatFlags');
            obj.IncompatFlags = uint8(val);
        end
        function set.CompatFlags(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Mavlink', 'CompatFlags');
            obj.CompatFlags = uint8(val);
        end
        function set.Seq(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Mavlink', 'Seq');
            obj.Seq = uint8(val);
        end
        function set.Sysid(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Mavlink', 'Sysid');
            obj.Sysid = uint8(val);
        end
        function set.Compid(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Mavlink', 'Compid');
            obj.Compid = uint8(val);
        end
        function set.Msgid(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Mavlink', 'Msgid');
            obj.Msgid = uint32(val);
        end
        function set.Checksum(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Mavlink', 'Checksum');
            obj.Checksum = uint16(val);
        end
        function set.Payload64(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = uint64.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'Mavlink', 'Payload64');
            obj.Payload64 = uint64(val);
        end
        function set.Signature(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = uint8.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'Mavlink', 'Signature');
            obj.Signature = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.mavros_msgs.Mavlink.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.mavros_msgs.Mavlink(strObj);
        end
    end
end