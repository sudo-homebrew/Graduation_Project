
classdef AveragingAction < ros.Message
    %AveragingAction MATLAB implementation of actionlib_tutorials/AveragingAction
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'actionlib_tutorials/AveragingAction' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '628678f2b4fa6a5951746a4a2d39e716' % The MD5 Checksum of the message definition
        PropertyList = { 'ActionGoal' 'ActionResult' 'ActionFeedback' } % List of non-constant message properties
        ROSPropertyList = { 'action_goal' 'action_result' 'action_feedback' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.actionlib_tutorials.AveragingActionGoal' ...
            'ros.msggen.actionlib_tutorials.AveragingActionResult' ...
            'ros.msggen.actionlib_tutorials.AveragingActionFeedback' ...
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
            validClasses = {'ros.msggen.actionlib_tutorials.AveragingActionGoal'};
            validateattributes(val, validClasses, validAttributes, 'AveragingAction', 'ActionGoal')
            obj.ActionGoal = val;
        end
        function set.ActionResult(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.actionlib_tutorials.AveragingActionResult'};
            validateattributes(val, validClasses, validAttributes, 'AveragingAction', 'ActionResult')
            obj.ActionResult = val;
        end
        function set.ActionFeedback(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.actionlib_tutorials.AveragingActionFeedback'};
            validateattributes(val, validClasses, validAttributes, 'AveragingAction', 'ActionFeedback')
            obj.ActionFeedback = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.actionlib_tutorials.AveragingAction.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.actionlib_tutorials.AveragingAction(strObj);
        end
    end
end
