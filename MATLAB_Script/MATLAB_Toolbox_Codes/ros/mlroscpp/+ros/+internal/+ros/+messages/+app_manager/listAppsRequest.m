function [data, info] = listAppsRequest
%ListApps gives an empty data for app_manager/ListAppsRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'app_manager/ListAppsRequest';
info.MessageType = 'app_manager/ListAppsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
