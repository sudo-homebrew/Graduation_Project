
classdef TurtlebotMoveAction < ros.Message
    %TurtlebotMoveAction MATLAB implementation of turtlebot_actions/TurtlebotMoveAction
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'turtlebot_actions/TurtlebotMoveAction' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '339929f411dcfd188670046028b412ee' % The MD5 Checksum of the message definition
        PropertyList = { 'ActionGoal' 'ActionResult' 'ActionFeedback' } % List of non-constant message properties
        ROSPropertyList = { 'action_goal' 'action_result' 'action_feedback' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.turtlebot_actions.TurtlebotMoveActionGoal' ...
            'ros.msggen.turtlebot_actions.TurtlebotMoveActionResult' ...
            'ros.msggen.turtlebot_actions.TurtlebotMoveActionFeedback' ...
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
            validClasses = {'ros.msggen.turtlebot_actions.TurtlebotMoveActionGoal'};
            validateattributes(val, validClasses, validAttributes, 'TurtlebotMoveAction', 'ActionGoal')
            obj.ActionGoal = val;
        end
        function set.ActionResult(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.turtlebot_actions.TurtlebotMoveActionResult'};
            validateattributes(val, validClasses, validAttributes, 'TurtlebotMoveAction', 'ActionResult')
            obj.ActionResult = val;
        end
        function set.ActionFeedback(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.turtlebot_actions.TurtlebotMoveActionFeedback'};
            validateattributes(val, validClasses, validAttributes, 'TurtlebotMoveAction', 'ActionFeedback')
            obj.ActionFeedback = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.turtlebot_actions.TurtlebotMoveAction.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.turtlebot_actions.TurtlebotMoveAction(strObj);
        end
    end
end