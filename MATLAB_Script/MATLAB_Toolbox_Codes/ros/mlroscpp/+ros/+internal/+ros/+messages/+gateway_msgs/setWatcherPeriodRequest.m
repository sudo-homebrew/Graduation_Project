function [data, info] = setWatcherPeriodRequest
%SetWatcherPeriod gives an empty data for gateway_msgs/SetWatcherPeriodRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gateway_msgs/SetWatcherPeriodRequest';
[data.Period, info.Period] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'gateway_msgs/SetWatcherPeriodRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'period';
