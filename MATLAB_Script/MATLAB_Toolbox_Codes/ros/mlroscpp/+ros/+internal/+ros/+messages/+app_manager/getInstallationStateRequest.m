function [data, info] = getInstallationStateRequest
%GetInstallationState gives an empty data for app_manager/GetInstallationStateRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'app_manager/GetInstallationStateRequest';
[data.RemoteUpdate, info.RemoteUpdate] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'app_manager/GetInstallationStateRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'remote_update';
