
classdef ExecFootstepsResult < ros.Message
    %ExecFootstepsResult MATLAB implementation of humanoid_nav_msgs/ExecFootstepsResult
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'humanoid_nav_msgs/ExecFootstepsResult' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '5dfde2cb244d6c76567d3c52c40a988c' % The MD5 Checksum of the message definition
        PropertyList = { 'ExecutedFootsteps' } % List of non-constant message properties
        ROSPropertyList = { 'executed_footsteps' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.humanoid_nav_msgs.StepTarget' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        ExecutedFootsteps
    end
    methods
        function set.ExecutedFootsteps(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.humanoid_nav_msgs.StepTarget.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.humanoid_nav_msgs.StepTarget'};
            validateattributes(val, validClasses, validAttributes, 'ExecFootstepsResult', 'ExecutedFootsteps')
            obj.ExecutedFootsteps = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.humanoid_nav_msgs.ExecFootstepsResult.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.humanoid_nav_msgs.ExecFootstepsResult(strObj);
        end
    end
end