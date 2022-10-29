classdef  MsgListOutputStream < ros.slros.internal.sim.MsgOutputStream
%This class is for internal use only. It may be removed in the future.

%  MsgListOutputStream provides a sink of ROS messages (the messages
%  saved to an underlying list that is inspectable by a separate
%  method). It is analogous to an ostringstream in C++, or a string-based
%  OutputStream in Java.

%  Copyright 2014-2018 The MathWorks, Inc.

    properties
        Topic
        MessageType
    end

    properties(SetAccess=private)
        MessageList = {}
    end

    methods

        function obj = MsgListOutputStream(topic, msgType)
            obj.Topic = topic;
            obj.MessageType = msgType;
        end

        function resetMessageList(obj)
            obj.MessageList = {};
        end

        function sendMessage(obj, msg)
            obj.MessageList{end+1} = msg;
        end

    end

end
