
classdef MmRobotPosition < ros.Message
    %MmRobotPosition MATLAB implementation of adhoc_communication/MmRobotPosition
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'adhoc_communication/MmRobotPosition' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'b110eae90e8648401835f8c826926f86' % The MD5 Checksum of the message definition
        PropertyList = { 'Position' 'SrcRobot' } % List of non-constant message properties
        ROSPropertyList = { 'position' 'src_robot' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.geometry_msgs.PoseStamped' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Position
        SrcRobot
    end
    methods
        function set.Position(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.PoseStamped'};
            validateattributes(val, validClasses, validAttributes, 'MmRobotPosition', 'Position')
            obj.Position = val;
        end
        function set.SrcRobot(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'MmRobotPosition', 'SrcRobot');
            obj.SrcRobot = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.adhoc_communication.MmRobotPosition.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.adhoc_communication.MmRobotPosition(strObj);
        end
    end
end