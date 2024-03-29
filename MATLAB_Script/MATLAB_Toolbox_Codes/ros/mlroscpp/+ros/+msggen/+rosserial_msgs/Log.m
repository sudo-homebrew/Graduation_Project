
classdef Log < ros.Message
    %Log MATLAB implementation of rosserial_msgs/Log
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rosserial_msgs/Log' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '11abd731c25933261cd6183bd12d6295' % The MD5 Checksum of the message definition
        PropertyList = { 'Level' 'Msg' } % List of non-constant message properties
        ROSPropertyList = { 'level' 'msg' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        ROSDEBUG = uint8(0)
        INFO = uint8(1)
        WARN = uint8(2)
        ERROR = uint8(3)
        FATAL = uint8(4)
    end
    properties
        Level
        Msg
    end
    methods
        function set.Level(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Log', 'Level');
            obj.Level = uint8(val);
        end
        function set.Msg(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Log', 'Msg');
            obj.Msg = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rosserial_msgs.Log.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rosserial_msgs.Log(strObj);
        end
    end
end
