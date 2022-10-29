function [data, info] = getAppListRequest
%GetAppList gives an empty data for rocon_app_manager_msgs/GetAppListRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_app_manager_msgs/GetAppListRequest';
info.MessageType = 'rocon_app_manager_msgs/GetAppListRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
