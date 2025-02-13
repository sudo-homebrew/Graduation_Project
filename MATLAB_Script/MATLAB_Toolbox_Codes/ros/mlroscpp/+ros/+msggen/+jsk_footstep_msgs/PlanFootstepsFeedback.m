
classdef PlanFootstepsFeedback < ros.Message
    %PlanFootstepsFeedback MATLAB implementation of jsk_footstep_msgs/PlanFootstepsFeedback
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'jsk_footstep_msgs/PlanFootstepsFeedback' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '4d6b3eec05cb69fcc42319013eb20a28' % The MD5 Checksum of the message definition
        PropertyList = { 'Feedback' } % List of non-constant message properties
        ROSPropertyList = { 'feedback' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.jsk_footstep_msgs.FootstepArray' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Feedback
    end
    methods
        function set.Feedback(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.jsk_footstep_msgs.FootstepArray'};
            validateattributes(val, validClasses, validAttributes, 'PlanFootstepsFeedback', 'Feedback')
            obj.Feedback = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.jsk_footstep_msgs.PlanFootstepsFeedback.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.jsk_footstep_msgs.PlanFootstepsFeedback(strObj);
        end
    end
end
