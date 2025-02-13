
classdef ShowGraspsAction < ros.Message
    %ShowGraspsAction MATLAB implementation of cob_grasp_generation/ShowGraspsAction
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'cob_grasp_generation/ShowGraspsAction' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '156494c0414432ecd331d6133324b528' % The MD5 Checksum of the message definition
        PropertyList = { 'ActionGoal' 'ActionResult' 'ActionFeedback' } % List of non-constant message properties
        ROSPropertyList = { 'action_goal' 'action_result' 'action_feedback' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.cob_grasp_generation.ShowGraspsActionGoal' ...
            'ros.msggen.cob_grasp_generation.ShowGraspsActionResult' ...
            'ros.msggen.cob_grasp_generation.ShowGraspsActionFeedback' ...
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
            validClasses = {'ros.msggen.cob_grasp_generation.ShowGraspsActionGoal'};
            validateattributes(val, validClasses, validAttributes, 'ShowGraspsAction', 'ActionGoal')
            obj.ActionGoal = val;
        end
        function set.ActionResult(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.cob_grasp_generation.ShowGraspsActionResult'};
            validateattributes(val, validClasses, validAttributes, 'ShowGraspsAction', 'ActionResult')
            obj.ActionResult = val;
        end
        function set.ActionFeedback(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.cob_grasp_generation.ShowGraspsActionFeedback'};
            validateattributes(val, validClasses, validAttributes, 'ShowGraspsAction', 'ActionFeedback')
            obj.ActionFeedback = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.cob_grasp_generation.ShowGraspsAction.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.cob_grasp_generation.ShowGraspsAction(strObj);
        end
    end
end
