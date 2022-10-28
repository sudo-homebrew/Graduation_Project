function [data, info] = constants
%Constants gives an empty data for rocon_app_manager_msgs/Constants

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_app_manager_msgs/Constants';
[data.NOREMOTECONNECTION, info.NOREMOTECONNECTION] = ros.internal.ros.messages.ros.char('string',0);
[data.NOREMOTECONNECTION, info.NOREMOTECONNECTION] = ros.internal.ros.messages.ros.char('string',1,'none');
[data.APPSTOPPED, info.APPSTOPPED] = ros.internal.ros.messages.ros.char('string',0);
[data.APPSTOPPED, info.APPSTOPPED] = ros.internal.ros.messages.ros.char('string',1,'stopped');
[data.APPRUNNING, info.APPRUNNING] = ros.internal.ros.messages.ros.char('string',0);
[data.APPRUNNING, info.APPRUNNING] = ros.internal.ros.messages.ros.char('string',1,'running');
info.MessageType = 'rocon_app_manager_msgs/Constants';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'NO_REMOTE_CONNECTION';
info.MatPath{2} = 'APP_STOPPED';
info.MatPath{3} = 'APP_RUNNING';
