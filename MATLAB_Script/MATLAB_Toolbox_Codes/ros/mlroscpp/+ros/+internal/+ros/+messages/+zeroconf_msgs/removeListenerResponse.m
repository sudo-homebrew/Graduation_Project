function [data, info] = removeListenerResponse
%RemoveListener gives an empty data for zeroconf_msgs/RemoveListenerResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'zeroconf_msgs/RemoveListenerResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'zeroconf_msgs/RemoveListenerResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'result';