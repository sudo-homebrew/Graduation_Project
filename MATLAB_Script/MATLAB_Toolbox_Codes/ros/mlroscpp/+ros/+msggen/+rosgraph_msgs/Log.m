
classdef Log < ros.Message
    %Log MATLAB implementation of rosgraph_msgs/Log
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'rosgraph_msgs/Log' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'acffd30cd6b6de30f120938c17c593fb' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Level' 'Name' 'Msg' 'File' 'Function' 'Line' 'Topics' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'level' 'name' 'msg' 'file' 'function' 'line' 'topics' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
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
        DEBUG = int8(1)
        INFO = int8(2)
        WARN = int8(4)
        ERROR = int8(8)
        FATAL = int8(16)
    end
    properties
        Header
        Level
        Name
        Msg
        File
        Function
        Line
        Topics
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'Log', 'Header')
            obj.Header = val;
        end
        function set.Level(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Log', 'Level');
            obj.Level = int8(val);
        end
        function set.Name(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Log', 'Name');
            obj.Name = char(val);
        end
        function set.Msg(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Log', 'Msg');
            obj.Msg = char(val);
        end
        function set.File(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Log', 'File');
            obj.File = char(val);
        end
        function set.Function(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Log', 'Function');
            obj.Function = char(val);
        end
        function set.Line(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Log', 'Line');
            obj.Line = uint32(val);
        end
        function set.Topics(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'Log', 'Topics');
            obj.Topics = cell(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.rosgraph_msgs.Log.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.rosgraph_msgs.Log(strObj);
        end
    end
end