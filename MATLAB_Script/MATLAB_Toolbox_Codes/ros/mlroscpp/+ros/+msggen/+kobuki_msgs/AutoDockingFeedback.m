
classdef AutoDockingFeedback < ros.Message
    %AutoDockingFeedback MATLAB implementation of kobuki_msgs/AutoDockingFeedback
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'kobuki_msgs/AutoDockingFeedback' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '03343b6aa0067ce6251bcc08bf318388' % The MD5 Checksum of the message definition
        PropertyList = { 'State' 'Text' } % List of non-constant message properties
        ROSPropertyList = { 'state' 'text' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        State
        Text
    end
    methods
        function set.State(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'AutoDockingFeedback', 'State');
            obj.State = char(val);
        end
        function set.Text(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'AutoDockingFeedback', 'Text');
            obj.Text = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.kobuki_msgs.AutoDockingFeedback.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.kobuki_msgs.AutoDockingFeedback(strObj);
        end
    end
end