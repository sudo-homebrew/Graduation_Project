classdef  MsgListInputStream < ros.slros.internal.sim.MsgInputStream
%This class is for internal use only. It may be removed in the future.

%  MsgListInputStream provides a source of ROS messages (pulling the
%  messages off an underlying list). It is analogous to an istringstream
%  in C++, or a string-based InputStream in Java.

%  Copyright 2014-2018 The MathWorks, Inc.

    properties
        Topic
        MessageType
    end


    properties (Access=private)
        MessageList = { [] } % cell array of messages. [] indicates "no message"
        BlankMessage
    end

    properties (SetAccess=private, Transient)
        CurrentMessageIndex = 1
        LastValidMsgIndex = -1
    end


    methods


        function obj = MsgListInputStream(topic, msgType, nodeObj)   %#ok<INUSD>
        % nodeObj is ignored by design
            obj.Topic = topic;
            obj.MessageType = msgType;
        end


        function setMessageList(obj, messageList)
        % This is not a setter as a set method cannot change another
        % property (CurrentMsgIndex, in this case)
            validateattributes(messageList, {'cell'}, {'nonempty'});
            for i=1:numel(messageList)
                if ~(isa(messageList{i}, 'ros.Message') || isempty(messageList{i}))
                    error(message('ros:slros:msgstream:InvalidMsgListInput', 'setMessageList'));
                end
            end
            obj.MessageList = messageList;
            obj.CurrentMessageIndex = 1;
            obj.LastValidMsgIndex = -1;

            validMsgIndex = find(~cellfun(@isempty, messageList), 1, 'first');
            obj.BlankMessage = rosmessage(messageList{validMsgIndex}.MessageType);
        end


        function [isNewMsg, msg] = getLatestMessage(obj)
            msg = obj.MessageList{ obj.CurrentMessageIndex };
            if isempty(msg)
                isNewMsg = false;
                if obj.LastValidMsgIndex > 0
                    msg = obj.MessageList{ obj.LastValidMsgIndex };
                else
                    msg = obj.BlankMessage;
                end
            else % really new msg
                isNewMsg = true;
                obj.LastValidMsgIndex = obj.CurrentMessageIndex;
            end
            obj.CurrentMessageIndex = mod(obj.CurrentMessageIndex, numel(obj.MessageList)) + 1;
        end

        %%

    end


end
