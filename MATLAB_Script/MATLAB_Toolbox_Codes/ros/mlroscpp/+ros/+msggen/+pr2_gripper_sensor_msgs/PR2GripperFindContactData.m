
classdef PR2GripperFindContactData < ros.Message
    %PR2GripperFindContactData MATLAB implementation of pr2_gripper_sensor_msgs/PR2GripperFindContactData
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'pr2_gripper_sensor_msgs/PR2GripperFindContactData' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'bc53e3dc7d19b896ca9b5ea205d54b91' % The MD5 Checksum of the message definition
        PropertyList = { 'Stamp' 'Rtstate' 'ContactConditionsMet' 'LeftFingertipPadContact' 'RightFingertipPadContact' 'LeftFingertipPadForce' 'RightFingertipPadForce' 'JointPosition' 'JointEffort' } % List of non-constant message properties
        ROSPropertyList = { 'stamp' 'rtstate' 'contact_conditions_met' 'left_fingertip_pad_contact' 'right_fingertip_pad_contact' 'left_fingertip_pad_force' 'right_fingertip_pad_force' 'joint_position' 'joint_effort' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msg.Time' ...
            'ros.msggen.pr2_gripper_sensor_msgs.PR2GripperSensorRTState' ...
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
        Stamp
        Rtstate
        ContactConditionsMet
        LeftFingertipPadContact
        RightFingertipPadContact
        LeftFingertipPadForce
        RightFingertipPadForce
        JointPosition
        JointEffort
    end
    methods
        function set.Stamp(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.Time'};
            validateattributes(val, validClasses, validAttributes, 'PR2GripperFindContactData', 'Stamp')
            obj.Stamp = val;
        end
        function set.Rtstate(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.pr2_gripper_sensor_msgs.PR2GripperSensorRTState'};
            validateattributes(val, validClasses, validAttributes, 'PR2GripperFindContactData', 'Rtstate')
            obj.Rtstate = val;
        end
        function set.ContactConditionsMet(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PR2GripperFindContactData', 'ContactConditionsMet');
            obj.ContactConditionsMet = logical(val);
        end
        function set.LeftFingertipPadContact(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PR2GripperFindContactData', 'LeftFingertipPadContact');
            obj.LeftFingertipPadContact = logical(val);
        end
        function set.RightFingertipPadContact(obj, val)
            validClasses = {'logical', 'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PR2GripperFindContactData', 'RightFingertipPadContact');
            obj.RightFingertipPadContact = logical(val);
        end
        function set.LeftFingertipPadForce(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PR2GripperFindContactData', 'LeftFingertipPadForce');
            obj.LeftFingertipPadForce = double(val);
        end
        function set.RightFingertipPadForce(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PR2GripperFindContactData', 'RightFingertipPadForce');
            obj.RightFingertipPadForce = double(val);
        end
        function set.JointPosition(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PR2GripperFindContactData', 'JointPosition');
            obj.JointPosition = double(val);
        end
        function set.JointEffort(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'PR2GripperFindContactData', 'JointEffort');
            obj.JointEffort = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.pr2_gripper_sensor_msgs.PR2GripperFindContactData.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.pr2_gripper_sensor_msgs.PR2GripperFindContactData(strObj);
        end
    end
end