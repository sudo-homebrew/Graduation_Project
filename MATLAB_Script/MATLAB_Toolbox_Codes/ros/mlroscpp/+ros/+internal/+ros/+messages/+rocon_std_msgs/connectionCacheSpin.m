function [data, info] = connectionCacheSpin
%ConnectionCacheSpin gives an empty data for rocon_std_msgs/ConnectionCacheSpin

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'rocon_std_msgs/ConnectionCacheSpin';
[data.SpinFreq, info.SpinFreq] = ros.internal.ros.messages.ros.default_type('single',1);
[data.SpinTimer, info.SpinTimer] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'rocon_std_msgs/ConnectionCacheSpin';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'spin_freq';
info.MatPath{2} = 'spin_timer';
