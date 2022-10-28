function [data, info] = errorCodes
%ErrorCodes gives an empty data for rocon_app_manager_msgs/ErrorCodes

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_app_manager_msgs/ErrorCodes';
[data.SUCCESS, info.SUCCESS] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.MULTIRAPPNOTSUPPORTED, info.MULTIRAPPNOTSUPPORTED] = ros.internal.ros.messages.ros.default_type('int8',1, 10);
[data.RAPPISNOTRUNNING, info.RAPPISNOTRUNNING] = ros.internal.ros.messages.ros.default_type('int8',1, 20);
info.MessageType = 'rocon_app_manager_msgs/ErrorCodes';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'SUCCESS';
info.MatPath{2} = 'MULTI_RAPP_NOT_SUPPORTED';
info.MatPath{3} = 'RAPP_IS_NOT_RUNNING';
