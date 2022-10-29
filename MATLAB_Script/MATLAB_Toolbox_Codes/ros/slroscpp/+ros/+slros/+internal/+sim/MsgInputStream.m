classdef MsgInputStream < handle
%This class is for internal use only. It may be removed in the future.

%  MsgInputStream - An interface for a source of ROS messages

%  Copyright 2014-2018 The MathWorks, Inc.

    properties (Abstract)
        Topic
        MessageType
    end


    methods (Abstract)
        [isNewMsg, msg] = getLatestMessage(obj)
    end

end
