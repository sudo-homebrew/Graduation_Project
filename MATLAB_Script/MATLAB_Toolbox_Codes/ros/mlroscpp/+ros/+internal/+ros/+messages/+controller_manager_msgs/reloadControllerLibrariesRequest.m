function [data, info] = reloadControllerLibrariesRequest
%ReloadControllerLibraries gives an empty data for controller_manager_msgs/ReloadControllerLibrariesRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'controller_manager_msgs/ReloadControllerLibrariesRequest';
[data.ForceKill, info.ForceKill] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'controller_manager_msgs/ReloadControllerLibrariesRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'force_kill';
