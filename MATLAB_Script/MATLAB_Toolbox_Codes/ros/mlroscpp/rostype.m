classdef rostype < ros.msg.internal.ROSMsgConsts & ros.msg.internal.ROSCustomMsgConsts
%ROSTYPE Access available ROS message types.
%   ROSTYPE allows you to browse the list of available message types,
%   so you can use tab completion and do not have to rely on typing
%   error-free message type strings.
%
%   Example:
%      t = ROSTYPE.std_msgs_String
%      msg = rosmessage(ROSTYPE.sensor_msgs_PointCloud2);

%   Copyright 2020-2021 The MathWorks, Inc.

    methods (Static)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            msgTypes = ros.msg.internal.ROSMsgConsts.getMessageList;
            customMsgTypes = ros.msg.internal.ROSCustomMsgConsts.getMessageList;
            reg = ros.internal.CustomMessageRegistry.getInstance('ros');
            msgTypes = (msgTypes)';
            customMsgTypes = (customMsgTypes)';
            msgTypes = sort([msgTypes customMsgTypes]);
            msgList = sort([msgTypes, reg.getMessageList()]);
            msgList = reshape(msgList,[],1);
            messageList = msgList;
        end

        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            svcTypes = ros.msg.internal.ROSMsgConsts.getServiceList;
            customSrvTypes = ros.msg.internal.ROSCustomMsgConsts.getServiceList;
            reg = ros.internal.CustomMessageRegistry.getInstance('ros');
            svcTypes = (svcTypes)';
            customSrvTypes = (customSrvTypes)';
            svcTypes = sort([svcTypes customSrvTypes]);
            svcList = sort([svcTypes, reg.getServiceList()]);
            svcList = reshape(svcList,[],1);
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            actionTypes = ros.msg.internal.ROSMsgConsts.getActionList;
            customActionTypes = ros.msg.internal.ROSCustomMsgConsts.getActionList;
            reg = ros.internal.CustomMessageRegistry.getInstance('ros');
            actionTypes = (actionTypes)';
            customActionTypes = (customActionTypes)';
            actionTypes = sort([actionTypes customActionTypes]);
            actionList = sort([actionTypes, reg.getActionList()]);
            actionList = reshape(actionList,[],1);
        end
    end
end
