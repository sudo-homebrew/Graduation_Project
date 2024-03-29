
classdef UInt16MultiArray < ros.Message
    %UInt16MultiArray MATLAB implementation of std_msgs/UInt16MultiArray
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'std_msgs/UInt16MultiArray' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '52f264f1c973c4b73790d384c6cb4484' % The MD5 Checksum of the message definition
        PropertyList = { 'Layout' 'Data' } % List of non-constant message properties
        ROSPropertyList = { 'layout' 'data' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.MultiArrayLayout' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Layout
        Data
    end
    methods
        function set.Layout(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.MultiArrayLayout'};
            validateattributes(val, validClasses, validAttributes, 'UInt16MultiArray', 'Layout')
            obj.Layout = val;
        end
        function set.Data(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = uint16.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'UInt16MultiArray', 'Data');
            obj.Data = uint16(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.std_msgs.UInt16MultiArray.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.std_msgs.UInt16MultiArray(strObj);
        end
    end
end
