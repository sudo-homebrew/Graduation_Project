
classdef JointControl < ros.Message
    %JointControl MATLAB implementation of r2_msgs/JointControl
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'r2_msgs/JointControl' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e38c02f4ffdedb57646a4576b7e0ccf6' % The MD5 Checksum of the message definition
        PropertyList = { 'Publisher' 'TimeStamp' 'Joint' 'RegisterValue' 'EnableBridge' 'EnableMotor' 'ReleaseBrake' 'EmbeddedMotCom' 'ControlMode' 'ClearFaults' } % List of non-constant message properties
        ROSPropertyList = { 'publisher' 'timeStamp' 'joint' 'registerValue' 'enableBridge' 'enableMotor' 'releaseBrake' 'embeddedMotCom' 'controlMode' 'clearFaults' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
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
    end
    properties
        Publisher
        TimeStamp
        Joint
        RegisterValue
        EnableBridge
        EnableMotor
        ReleaseBrake
        EmbeddedMotCom
        ControlMode
        ClearFaults
    end
    methods
        function set.Publisher(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'JointControl', 'Publisher');
            obj.Publisher = char(val);
        end
        function set.TimeStamp(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControl', 'TimeStamp');
            obj.TimeStamp = double(val);
        end
        function set.Joint(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'JointControl', 'Joint');
            obj.Joint = char(val);
        end
        function set.RegisterValue(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControl', 'RegisterValue');
            obj.RegisterValue = uint32(val);
        end
        function set.EnableBridge(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControl', 'EnableBridge');
            obj.EnableBridge = logical(val);
        end
        function set.EnableMotor(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControl', 'EnableMotor');
            obj.EnableMotor = logical(val);
        end
        function set.ReleaseBrake(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControl', 'ReleaseBrake');
            obj.ReleaseBrake = logical(val);
        end
        function set.EmbeddedMotCom(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControl', 'EmbeddedMotCom');
            obj.EmbeddedMotCom = logical(val);
        end
        function set.ControlMode(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControl', 'ControlMode');
            obj.ControlMode = uint16(val);
        end
        function set.ClearFaults(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'JointControl', 'ClearFaults');
            obj.ClearFaults = logical(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.r2_msgs.JointControl.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.r2_msgs.JointControl(strObj);
        end
    end
end