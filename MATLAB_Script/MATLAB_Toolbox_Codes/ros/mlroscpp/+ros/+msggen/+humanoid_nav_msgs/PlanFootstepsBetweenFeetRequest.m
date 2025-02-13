
classdef PlanFootstepsBetweenFeetRequest < ros.Message
    %PlanFootstepsBetweenFeetRequest MATLAB implementation of humanoid_nav_msgs/PlanFootstepsBetweenFeetRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'humanoid_nav_msgs/PlanFootstepsBetweenFeetRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'a081711eb51a4a4283b2b9f345efe272' % The MD5 Checksum of the message definition
        PropertyList = { 'StartLeft' 'StartRight' 'GoalLeft' 'GoalRight' } % List of non-constant message properties
        ROSPropertyList = { 'start_left' 'start_right' 'goal_left' 'goal_right' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.humanoid_nav_msgs.StepTarget' ...
            'ros.msggen.humanoid_nav_msgs.StepTarget' ...
            'ros.msggen.humanoid_nav_msgs.StepTarget' ...
            'ros.msggen.humanoid_nav_msgs.StepTarget' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        StartLeft
        StartRight
        GoalLeft
        GoalRight
    end
    methods
        function set.StartLeft(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.humanoid_nav_msgs.StepTarget'};
            validateattributes(val, validClasses, validAttributes, 'PlanFootstepsBetweenFeetRequest', 'StartLeft')
            obj.StartLeft = val;
        end
        function set.StartRight(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.humanoid_nav_msgs.StepTarget'};
            validateattributes(val, validClasses, validAttributes, 'PlanFootstepsBetweenFeetRequest', 'StartRight')
            obj.StartRight = val;
        end
        function set.GoalLeft(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.humanoid_nav_msgs.StepTarget'};
            validateattributes(val, validClasses, validAttributes, 'PlanFootstepsBetweenFeetRequest', 'GoalLeft')
            obj.GoalLeft = val;
        end
        function set.GoalRight(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.humanoid_nav_msgs.StepTarget'};
            validateattributes(val, validClasses, validAttributes, 'PlanFootstepsBetweenFeetRequest', 'GoalRight')
            obj.GoalRight = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.humanoid_nav_msgs.PlanFootstepsBetweenFeetRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.humanoid_nav_msgs.PlanFootstepsBetweenFeetRequest(strObj);
        end
    end
end
