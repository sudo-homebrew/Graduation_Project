
classdef WheelDropEvent < ros.Message
    %WheelDropEvent MATLAB implementation of kobuki_msgs/WheelDropEvent
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'kobuki_msgs/WheelDropEvent' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e102837d89384d67669a0df86b63f33b' % The MD5 Checksum of the message definition
        PropertyList = { 'Wheel' 'State' } % List of non-constant message properties
        ROSPropertyList = { 'wheel' 'state' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        LEFT = uint8(0)
        RIGHT = uint8(1)
        RAISED = uint8(0)
        DROPPED = uint8(1)
    end
    properties
        Wheel
        State
    end
    methods
        function set.Wheel(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'WheelDropEvent', 'Wheel');
            obj.Wheel = uint8(val);
        end
        function set.State(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'WheelDropEvent', 'State');
            obj.State = uint8(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.kobuki_msgs.WheelDropEvent.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.kobuki_msgs.WheelDropEvent(strObj);
        end
    end
end