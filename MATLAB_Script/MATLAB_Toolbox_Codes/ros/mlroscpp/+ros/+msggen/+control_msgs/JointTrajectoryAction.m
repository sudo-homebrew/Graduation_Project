
classdef JointTrajectoryAction < ros.Message
    %JointTrajectoryAction MATLAB implementation of control_msgs/JointTrajectoryAction
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'control_msgs/JointTrajectoryAction' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'a04ba3ee8f6a2d0985a6aeaf23d9d7ad' % The MD5 Checksum of the message definition
        PropertyList = { 'ActionGoal' 'ActionResult' 'ActionFeedback' } % List of non-constant message properties
        ROSPropertyList = { 'action_goal' 'action_result' 'action_feedback' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.control_msgs.JointTrajectoryActionGoal' ...
            'ros.msggen.control_msgs.JointTrajectoryActionResult' ...
            'ros.msggen.control_msgs.JointTrajectoryActionFeedback' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ActionGoal
        ActionResult
        ActionFeedback
    end
    methods
        function set.ActionGoal(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.control_msgs.JointTrajectoryActionGoal'};
            validateattributes(val, validClasses, validAttributes, 'JointTrajectoryAction', 'ActionGoal')
            obj.ActionGoal = val;
        end
        function set.ActionResult(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.control_msgs.JointTrajectoryActionResult'};
            validateattributes(val, validClasses, validAttributes, 'JointTrajectoryAction', 'ActionResult')
            obj.ActionResult = val;
        end
        function set.ActionFeedback(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.control_msgs.JointTrajectoryActionFeedback'};
            validateattributes(val, validClasses, validAttributes, 'JointTrajectoryAction', 'ActionFeedback')
            obj.ActionFeedback = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.control_msgs.JointTrajectoryAction.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.control_msgs.JointTrajectoryAction(strObj);
        end
    end
end