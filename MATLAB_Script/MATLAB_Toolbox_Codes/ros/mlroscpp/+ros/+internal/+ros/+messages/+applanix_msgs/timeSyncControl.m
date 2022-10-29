function [data, info] = timeSyncControl
%TimeSyncControl gives an empty data for applanix_msgs/TimeSyncControl

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'applanix_msgs/TimeSyncControl';
[data.Transaction, info.Transaction] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.UserPpsTime, info.UserPpsTime] = ros.internal.ros.messages.ros.default_type('double',1);
[data.UserTimeConversionFactor, info.UserTimeConversionFactor] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'applanix_msgs/TimeSyncControl';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'transaction';
info.MatPath{2} = 'user_pps_time';
info.MatPath{3} = 'user_time_conversion_factor';
