
classdef GetMapAction < ros.Message
    %GetMapAction MATLAB implementation of nav_msgs/GetMapAction
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'nav_msgs/GetMapAction' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'e611ad23fbf237c031b7536416dc7cd7' % The MD5 Checksum of the message definition
        PropertyList = { 'ActionGoal' 'ActionResult' 'ActionFeedback' } % List of non-constant message properties
        ROSPropertyList = { 'action_goal' 'action_result' 'action_feedback' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.nav_msgs.GetMapActionGoal' ...
            'ros.msggen.nav_msgs.GetMapActionResult' ...
            'ros.msggen.nav_msgs.GetMapActionFeedback' ...
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
            validClasses = {'ros.msggen.nav_msgs.GetMapActionGoal'};
            validateattributes(val, validClasses, validAttributes, 'GetMapAction', 'ActionGoal')
            obj.ActionGoal = val;
        end
        function set.ActionResult(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.nav_msgs.GetMapActionResult'};
            validateattributes(val, validClasses, validAttributes, 'GetMapAction', 'ActionResult')
            obj.ActionResult = val;
        end
        function set.ActionFeedback(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.nav_msgs.GetMapActionFeedback'};
            validateattributes(val, validClasses, validAttributes, 'GetMapAction', 'ActionFeedback')
            obj.ActionFeedback = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.nav_msgs.GetMapAction.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.nav_msgs.GetMapAction(strObj);
        end
    end
end