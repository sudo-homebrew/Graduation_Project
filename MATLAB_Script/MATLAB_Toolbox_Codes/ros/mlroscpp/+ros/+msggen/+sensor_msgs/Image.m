
classdef Image < ros.Message
    %Image MATLAB implementation of sensor_msgs/Image
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'sensor_msgs/Image' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '060021388200f6f0f447d0fcd9c64743' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Height' 'Width' 'Encoding' 'IsBigendian' 'Step' 'Data' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'height' 'width' 'encoding' 'is_bigendian' 'step' 'data' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Height
        Width
        Encoding
        IsBigendian
        Step
        Data
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'Image', 'Header')
            obj.Header = val;
        end
        function set.Height(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Image', 'Height');
            obj.Height = uint32(val);
        end
        function set.Width(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Image', 'Width');
            obj.Width = uint32(val);
        end
        function set.Encoding(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Image', 'Encoding');
            obj.Encoding = char(val);
        end
        function set.IsBigendian(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Image', 'IsBigendian');
            obj.IsBigendian = uint8(val);
        end
        function set.Step(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'Image', 'Step');
            obj.Step = uint32(val);
        end
        function set.Data(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = uint8.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'Image', 'Data');
            obj.Data = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.sensor_msgs.Image.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.sensor_msgs.Image(strObj);
        end
    end
end