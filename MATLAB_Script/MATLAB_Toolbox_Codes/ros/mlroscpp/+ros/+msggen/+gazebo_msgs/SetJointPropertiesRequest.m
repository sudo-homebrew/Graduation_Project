
classdef SetJointPropertiesRequest < ros.Message
    %SetJointPropertiesRequest MATLAB implementation of gazebo_msgs/SetJointPropertiesRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'gazebo_msgs/SetJointPropertiesRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '331fd8f35fd27e3c1421175590258e26' % The MD5 Checksum of the message definition
        PropertyList = { 'OdeJointConfig' 'JointName' } % List of non-constant message properties
        ROSPropertyList = { 'ode_joint_config' 'joint_name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.gazebo_msgs.ODEJointProperties' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        OdeJointConfig
        JointName
    end
    methods
        function set.OdeJointConfig(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.gazebo_msgs.ODEJointProperties'};
            validateattributes(val, validClasses, validAttributes, 'SetJointPropertiesRequest', 'OdeJointConfig')
            obj.OdeJointConfig = val;
        end
        function set.JointName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'SetJointPropertiesRequest', 'JointName');
            obj.JointName = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.gazebo_msgs.SetJointPropertiesRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.gazebo_msgs.SetJointPropertiesRequest(strObj);
        end
    end
end