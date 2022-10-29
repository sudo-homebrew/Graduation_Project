function [data, info] = addListenerResponse
%AddListener gives an empty data for zeroconf_msgs/AddListenerResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'zeroconf_msgs/AddListenerResponse';
[data.Result, info.Result] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'zeroconf_msgs/AddListenerResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'result';
