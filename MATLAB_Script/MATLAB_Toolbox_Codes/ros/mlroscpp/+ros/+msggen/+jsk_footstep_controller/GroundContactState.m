
classdef GroundContactState < ros.Message
    %GroundContactState MATLAB implementation of jsk_footstep_controller/GroundContactState
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'jsk_footstep_controller/GroundContactState' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'da0f3906e0a6eafe324ba582440493ea' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'ContactState' 'ErrorPitchAngle' 'ErrorRollAngle' 'ErrorYawAngle' 'ErrorZ' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'contact_state' 'error_pitch_angle' 'error_roll_angle' 'error_yaw_angle' 'error_z' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
        CONTACTBOTHGROUND = uint8(1)
        CONTACTAIR = uint8(2)
        CONTACTLLEGGROUND = uint8(3)
        CONTACTRLEGGROUND = uint8(4)
        CONTACTUNSTABLE = uint8(5)
    end
    properties
        Header
        ContactState
        ErrorPitchAngle
        ErrorRollAngle
        ErrorYawAngle
        ErrorZ
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'GroundContactState', 'Header')
            obj.Header = val;
        end
        function set.ContactState(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'GroundContactState', 'ContactState');
            obj.ContactState = uint8(val);
        end
        function set.ErrorPitchAngle(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'GroundContactState', 'ErrorPitchAngle');
            obj.ErrorPitchAngle = double(val);
        end
        function set.ErrorRollAngle(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'GroundContactState', 'ErrorRollAngle');
            obj.ErrorRollAngle = double(val);
        end
        function set.ErrorYawAngle(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'GroundContactState', 'ErrorYawAngle');
            obj.ErrorYawAngle = double(val);
        end
        function set.ErrorZ(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'GroundContactState', 'ErrorZ');
            obj.ErrorZ = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.jsk_footstep_controller.GroundContactState.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.jsk_footstep_controller.GroundContactState(strObj);
        end
    end
end