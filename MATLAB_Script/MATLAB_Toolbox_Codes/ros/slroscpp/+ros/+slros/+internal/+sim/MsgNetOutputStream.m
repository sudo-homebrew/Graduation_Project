classdef  MsgNetOutputStream < ros.slros.internal.sim.MsgOutputStream
%This class is for internal use only. It may be removed in the future.

%  MsgNetOutputStream provides a sink of ROS messages (the messages are
%  sent to the ROS network via a Publisher). It is analogous to a
%  socket-based OutputStream in Java.

%  Copyright 2014-2018 The MathWorks, Inc.

    properties
        Topic
        MessageType
    end

    properties
        Publisher
    end


    methods

        function obj = MsgNetOutputStream(topic, msgType, nodeObj)
        % Do not canonicalize the topic (i.e., don't prepend a "/" in
        % front)
            obj.Topic = topic;
            obj.MessageType = msgType;
            obj.Publisher = ros.Publisher(nodeObj, obj.Topic, obj.MessageType);
        end

        function sendMessage(obj, msg)

            obj.Publisher.send(msg);
        end

    end


end
