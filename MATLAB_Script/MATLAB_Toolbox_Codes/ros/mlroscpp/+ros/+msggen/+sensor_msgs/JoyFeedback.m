
classdef JoyFeedback < ros.Message
    %JoyFeedback MATLAB implementation of sensor_msgs/JoyFeedback
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'sensor_msgs/JoyFeedback' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'f4dcd73460360d98f36e55ee7f2e46f1' % The MD5 Checksum of the message definition
        PropertyList = { 'Type' 'Id' 'Intensity' } % List of non-constant message properties
        ROSPropertyList = { 'type' 'id' 'intensity' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        TYPELED = uint8(0)
        TYPERUMBLE = uint8(1)
        TYPEBUZZER = uint8(2)
    end
    properties
        Type
        Id
        Intensity
    end
    methods
        function set.Type(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JoyFeedback', 'Type');
            obj.Type = uint8(val);
        end
        function set.Id(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JoyFeedback', 'Id');
            obj.Id = uint8(val);
        end
        function set.Intensity(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JoyFeedback', 'Intensity');
            obj.Intensity = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.sensor_msgs.JoyFeedback.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.sensor_msgs.JoyFeedback(strObj);
        end
    end
end