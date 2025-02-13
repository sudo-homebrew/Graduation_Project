
classdef Constraints < ros.Message
    %Constraints MATLAB implementation of moveit_msgs/Constraints
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'moveit_msgs/Constraints' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'cfd22a10c51e0dc2b28d98772d2b55d5' % The MD5 Checksum of the message definition
        PropertyList = { 'JointConstraints' 'PositionConstraints' 'OrientationConstraints' 'VisibilityConstraints' 'Name' } % List of non-constant message properties
        ROSPropertyList = { 'joint_constraints' 'position_constraints' 'orientation_constraints' 'visibility_constraints' 'name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.moveit_msgs.JointConstraint' ...
            'ros.msggen.moveit_msgs.PositionConstraint' ...
            'ros.msggen.moveit_msgs.OrientationConstraint' ...
            'ros.msggen.moveit_msgs.VisibilityConstraint' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        JointConstraints
        PositionConstraints
        OrientationConstraints
        VisibilityConstraints
        Name
    end
    methods
        function set.JointConstraints(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.moveit_msgs.JointConstraint.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.moveit_msgs.JointConstraint'};
            validateattributes(val, validClasses, validAttributes, 'Constraints', 'JointConstraints')
            obj.JointConstraints = val;
        end
        function set.PositionConstraints(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.moveit_msgs.PositionConstraint.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.moveit_msgs.PositionConstraint'};
            validateattributes(val, validClasses, validAttributes, 'Constraints', 'PositionConstraints')
            obj.PositionConstraints = val;
        end
        function set.OrientationConstraints(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.moveit_msgs.OrientationConstraint.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.moveit_msgs.OrientationConstraint'};
            validateattributes(val, validClasses, validAttributes, 'Constraints', 'OrientationConstraints')
            obj.OrientationConstraints = val;
        end
        function set.VisibilityConstraints(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.moveit_msgs.VisibilityConstraint.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.moveit_msgs.VisibilityConstraint'};
            validateattributes(val, validClasses, validAttributes, 'Constraints', 'VisibilityConstraints')
            obj.VisibilityConstraints = val;
        end
        function set.Name(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Constraints', 'Name');
            obj.Name = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.moveit_msgs.Constraints.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.moveit_msgs.Constraints(strObj);
        end
    end
end
