
classdef MultiDOFJointState < ros.Message
    %MultiDOFJointState MATLAB implementation of sensor_msgs/MultiDOFJointState
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'sensor_msgs/MultiDOFJointState' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '690f272f0640d2631c305eeb8301e59d' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Transforms' 'Twist' 'Wrench' 'JointNames' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'transforms' 'twist' 'wrench' 'joint_names' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.geometry_msgs.Transform' ...
            'ros.msggen.geometry_msgs.Twist' ...
            'ros.msggen.geometry_msgs.Wrench' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        Transforms
        Twist
        Wrench
        JointNames
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'MultiDOFJointState', 'Header')
            obj.Header = val;
        end
        function set.Transforms(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.geometry_msgs.Transform.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.geometry_msgs.Transform'};
            validateattributes(val, validClasses, validAttributes, 'MultiDOFJointState', 'Transforms')
            obj.Transforms = val;
        end
        function set.Twist(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.geometry_msgs.Twist.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.geometry_msgs.Twist'};
            validateattributes(val, validClasses, validAttributes, 'MultiDOFJointState', 'Twist')
            obj.Twist = val;
        end
        function set.Wrench(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.geometry_msgs.Wrench.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.geometry_msgs.Wrench'};
            validateattributes(val, validClasses, validAttributes, 'MultiDOFJointState', 'Wrench')
            obj.Wrench = val;
        end
        function set.JointNames(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'cell', 'string'};
            if isempty(val)
                % Allow empty [] input
                val = cell.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'MultiDOFJointState', 'JointNames');
            obj.JointNames = cell(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.sensor_msgs.MultiDOFJointState.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.sensor_msgs.MultiDOFJointState(strObj);
        end
    end
end