classdef  MsgOutputStream < handle
%This class is for internal use only. It may be removed in the future.

%  MsgOutputStream - An interface for a sink of ROS messages

%  Copyright 2014-2018 The MathWorks, Inc.


    properties (Abstract)
        Topic
        MessageType
    end


    methods (Abstract)
        sendMessage(obj, msg)
    end

end
