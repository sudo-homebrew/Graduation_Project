
classdef MoveGroupFeedback < ros.Message
    %MoveGroupFeedback MATLAB implementation of moveit_msgs/MoveGroupFeedback
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'moveit_msgs/MoveGroupFeedback' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'af6d3a99f0fbeb66d3248fa4b3e675fb' % The MD5 Checksum of the message definition
        PropertyList = { 'State' } % List of non-constant message properties
        ROSPropertyList = { 'state' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        State
    end
    methods
        function set.State(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'MoveGroupFeedback', 'State');
            obj.State = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.moveit_msgs.MoveGroupFeedback.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.moveit_msgs.MoveGroupFeedback(strObj);
        end
    end
end