
classdef PlatformName < ros.Message
    %PlatformName MATLAB implementation of clearpath_base/PlatformName
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'clearpath_base/PlatformName' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e541bb76a27a3e03e7987c32ec4fd724' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Name' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Name
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'PlatformName', 'Header')
            obj.Header = val;
        end
        function set.Name(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'PlatformName', 'Name');
            obj.Name = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.clearpath_base.PlatformName.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.clearpath_base.PlatformName(strObj);
        end
    end
end