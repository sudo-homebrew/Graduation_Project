function [data, info] = robustControllerStatus
%RobustControllerStatus gives an empty data for baxter_core_msgs/RobustControllerStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'baxter_core_msgs/RobustControllerStatus';
[data.IsEnabled, info.IsEnabled] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Complete, info.Complete] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.NOTCOMPLETE, info.NOTCOMPLETE] = ros.internal.ros.messages.ros.default_type('int32',1, 0);
[data.COMPLETEWFAILURE, info.COMPLETEWFAILURE] = ros.internal.ros.messages.ros.default_type('int32',1, 1);
[data.COMPLETEWSUCCESS, info.COMPLETEWSUCCESS] = ros.internal.ros.messages.ros.default_type('int32',1, 2);
[data.ControlUid, info.ControlUid] = ros.internal.ros.messages.ros.char('string',0);
[data.TimedOut, info.TimedOut] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ErrorCodes, info.ErrorCodes] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Labels, info.Labels] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'baxter_core_msgs/RobustControllerStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'isEnabled';
info.MatPath{2} = 'complete';
info.MatPath{3} = 'NOT_COMPLETE';
info.MatPath{4} = 'COMPLETE_W_FAILURE';
info.MatPath{5} = 'COMPLETE_W_SUCCESS';
info.MatPath{6} = 'controlUid';
info.MatPath{7} = 'timedOut';
info.MatPath{8} = 'errorCodes';
info.MatPath{9} = 'labels';
