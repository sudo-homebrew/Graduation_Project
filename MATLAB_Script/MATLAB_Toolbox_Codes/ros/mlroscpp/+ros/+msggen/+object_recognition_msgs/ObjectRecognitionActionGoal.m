
classdef ObjectRecognitionActionGoal < ros.Message
    %ObjectRecognitionActionGoal MATLAB implementation of object_recognition_msgs/ObjectRecognitionActionGoal
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'object_recognition_msgs/ObjectRecognitionActionGoal' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '195eff91387a5f42dbd13be53431366b' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'GoalId' 'Goal' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'goal_id' 'goal' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
            'ros.msggen.actionlib_msgs.GoalID' ...
            'ros.msggen.object_recognition_msgs.ObjectRecognitionGoal' ...
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
            validateattributes(val, validClasses, validAttributes, 'ObjectRecognitionActionGoal', 'Header')
            obj.Header = val;
        end
        function set.GoalId(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.actionlib_msgs.GoalID'};
            validateattributes(val, validClasses, validAttributes, 'ObjectRecognitionActionGoal', 'GoalId')
            obj.GoalId = val;
        end
        function set.Goal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.object_recognition_msgs.ObjectRecognitionGoal'};
            validateattributes(val, validClasses, validAttributes, 'ObjectRecognitionActionGoal', 'Goal')
            obj.Goal = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.object_recognition_msgs.ObjectRecognitionActionGoal.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.object_recognition_msgs.ObjectRecognitionActionGoal(strObj);
        end
    end
end