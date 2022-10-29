classdef GetCustomMessageList
%This class is for internal use only. It may be removed in the future.

% GetCustomMessageList Get the list custom message

%   Copyright 2019-2020 The MathWorks, Inc.

    properties
        MsgList = {'Custom Message not Found'};

    end

    methods
        function obj = GetCustomMessageList(~)
        % retrieve custom message list if importCustomMessageList is available
            if(robotics.slgazebo.internal.util.checkCustomMessageOnPath('importCustomMessageList.m'))
                TempList = importCustomMessageList;
                obj.MsgList = TempList.MsgList;
            end

        end
    end

end
