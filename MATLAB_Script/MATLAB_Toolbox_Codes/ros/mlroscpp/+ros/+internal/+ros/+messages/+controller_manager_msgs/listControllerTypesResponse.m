function [data, info] = listControllerTypesResponse
%ListControllerTypes gives an empty data for controller_manager_msgs/ListControllerTypesResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'controller_manager_msgs/ListControllerTypesResponse';
[data.Types, info.Types] = ros.internal.ros.messages.ros.char('string',NaN);
[data.BaseClasses, info.BaseClasses] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'controller_manager_msgs/ListControllerTypesResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'types';
info.MatPath{2} = 'base_classes';
