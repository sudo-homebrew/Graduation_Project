classdef  MsgNetInputStream <  ros.slros.internal.sim.MsgInputStream
%This class is for internal use only. It may be removed in the future.

%  MsgNetInputStream provides a source of ROS messages (pulling the
%  messages off a ROS network via a Subscriber). It is analogous to a
%  socket-based InputStream in Java.

%  Copyright 2014-2020 The MathWorks, Inc.

    properties
        Topic
        MessageType
    end

    properties % (Access=private, Transient)
        Subscriber
        LastSeenMsg
    end


    methods
        %%
        function obj = MsgNetInputStream(topic, msgType, nodeObj)
        % Do not canonicalize the topic (i.e., don't prepend a "/" in
        % front)
            obj.Topic = topic;
            obj.MessageType = msgType;
            obj.LastSeenMsg = rosmessage(msgType);
            if ~exist('nodeObj', 'var')
                obj.Subscriber = rossubscriber(obj.Topic, obj.MessageType);
            else
                obj.Subscriber = ros.Subscriber(nodeObj, obj.Topic, obj.MessageType);
            end
        end

        function [isNewMsg, msg] = getLatestMessage(obj)

            msg = obj.Subscriber.LatestMessage;
            if isempty(msg)
                isNewMsg = false;
                msg = obj.LastSeenMsg; % this is always guaranteed to be a valid message
            elseif isequal(msg, obj.LastSeenMsg)
                isNewMsg = false;
            else % really new msg
                isNewMsg = true;
                obj.LastSeenMsg = msg;
            end

        end

        %%

    end

end
