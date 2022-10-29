function [data, info] = stopAppRequest
%StopApp gives an empty data for rocon_app_manager_msgs/StopAppRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_app_manager_msgs/StopAppRequest';
info.MessageType = 'rocon_app_manager_msgs/StopAppRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,0);
