
classdef MuxDeleteRequest < ros.Message
    %MuxDeleteRequest MATLAB implementation of topic_tools/MuxDeleteRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.

    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'topic_tools/MuxDeleteRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd8f94bae31b356b24d0427f80426d0c3' % The MD5 Checksum of the message definition
        PropertyList = { 'Topic' } % List of non-constant message properties
        ROSPropertyList = { 'topic' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Topic
    end
    methods
        function set.Topic(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'MuxDeleteRequest', 'Topic');
            obj.Topic = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.topic_tools.MuxDeleteRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.topic_tools.MuxDeleteRequest(strObj);
        end
    end
end