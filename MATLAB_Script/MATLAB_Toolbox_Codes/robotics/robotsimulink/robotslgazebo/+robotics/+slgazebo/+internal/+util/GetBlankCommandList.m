classdef GetBlankCommandList
% This class is for internal use only. It may be removed in the future.

% GetBlankCommandList Get the list of blank command type

%   Copyright 2019-2020 The MathWorks, Inc.

    properties
        MsgList = {'ApplyLinkWrench',...
                   'ApplyJointTorque',...
                   'SetLinkWorldPose',...
                   'SetLinkLinearVelocity',...
                   'SetLinkAngularVelocity',...
                   'SetJointPosition',...
                   'SetJointVelocity'}
    end

    methods
        function obj = GetBlankCommandList(~)
        % if importCustomMessageList is available the
        % add custom message list to default commands
            if(robotics.slgazebo.internal.util.checkCustomMessageOnPath('importCustomMessageList.m'))
                TempList = importCustomMessageList;
                obj.MsgList = [obj.MsgList, TempList.MsgList];
            end
        end

    end
end
