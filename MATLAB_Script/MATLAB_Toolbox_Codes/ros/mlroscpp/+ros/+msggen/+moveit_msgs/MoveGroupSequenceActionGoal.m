
classdef MoveGroupSequenceActionGoal < ros.Message
    %MoveGroupSequenceActionGoal MATLAB implementation of moveit_msgs/MoveGroupSequenceActionGoal
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'moveit_msgs/MoveGroupSequenceActionGoal' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e8092a3ff0d4ffe7720dd24304ae29d9' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'GoalId' 'Goal' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'goal_id' 'goal' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.actionlib_msgs.GoalID' ...
            'ros.msggen.moveit_msgs.MoveGroupSequenceGoal' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Header
        GoalId
        Goal
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'MoveGroupSequenceActionGoal', 'Header')
            obj.Header = val;
        end
        function set.GoalId(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.actionlib_msgs.GoalID'};
            validateattributes(val, validClasses, validAttributes, 'MoveGroupSequenceActionGoal', 'GoalId')
            obj.GoalId = val;
        end
        function set.Goal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.moveit_msgs.MoveGroupSequenceGoal'};
            validateattributes(val, validClasses, validAttributes, 'MoveGroupSequenceActionGoal', 'Goal')
            obj.Goal = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.moveit_msgs.MoveGroupSequenceActionGoal.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.moveit_msgs.MoveGroupSequenceActionGoal(strObj);
        end
    end
end
