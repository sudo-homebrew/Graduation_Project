
classdef leap < ros.Message
    %leap MATLAB implementation of leap_motion/leap
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'leap_motion/leap' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '3e9a0dc7fd1a98f1d7489e9011c78807' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'HandDirection' 'HandNormal' 'HandPalmPos' 'HandPitch' 'HandRoll' 'HandYaw' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'hand_direction' 'hand_normal' 'hand_palm_pos' 'hand_pitch' 'hand_roll' 'hand_yaw' } % List of non-constant ROS message properties
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
        HandDirection
        HandNormal
        HandPalmPos
        HandPitch
        HandRoll
        HandYaw
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'leap', 'Header')
            obj.Header = val;
        end
        function set.HandDirection(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 3};
            validateattributes(val, validClasses, validAttributes, 'leap', 'HandDirection');
            obj.HandDirection = double(val);
        end
        function set.HandNormal(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 3};
            validateattributes(val, validClasses, validAttributes, 'leap', 'HandNormal');
            obj.HandNormal = double(val);
        end
        function set.HandPalmPos(obj, val)
            validClasses = {'numeric'};
            val = val(:);
            validAttributes = {'vector', 'numel', 3};
            validateattributes(val, validClasses, validAttributes, 'leap', 'HandPalmPos');
            obj.HandPalmPos = double(val);
        end
        function set.HandPitch(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'leap', 'HandPitch');
            obj.HandPitch = double(val);
        end
        function set.HandRoll(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'leap', 'HandRoll');
            obj.HandRoll = double(val);
        end
        function set.HandYaw(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'leap', 'HandYaw');
            obj.HandYaw = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.leap_motion.leap.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.leap_motion.leap(strObj);
        end
    end
end