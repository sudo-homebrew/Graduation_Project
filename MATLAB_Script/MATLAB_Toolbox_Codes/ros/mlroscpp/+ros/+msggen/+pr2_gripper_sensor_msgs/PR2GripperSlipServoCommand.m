
classdef PR2GripperSlipServoCommand < ros.Message
    %PR2GripperSlipServoCommand MATLAB implementation of pr2_gripper_sensor_msgs/PR2GripperSlipServoCommand
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'pr2_gripper_sensor_msgs/PR2GripperSlipServoCommand' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd41d8cd98f00b204e9800998ecf8427e' % The MD5 Checksum of the message definition
        PropertyList = {} % List of non-constant message properties
        ROSPropertyList = {} % List of non-constant ROS message properties
        PropertyMessageTypes = {} % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
    end
    methods
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.pr2_gripper_sensor_msgs.PR2GripperSlipServoCommand.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.pr2_gripper_sensor_msgs.PR2GripperSlipServoCommand(strObj);
        end
    end
end