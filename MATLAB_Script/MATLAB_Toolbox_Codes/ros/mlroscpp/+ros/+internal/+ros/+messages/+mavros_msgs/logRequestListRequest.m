function [data, info] = logRequestListRequest
%LogRequestList gives an empty data for mavros_msgs/LogRequestListRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/LogRequestListRequest';
[data.Start, info.Start] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.End, info.End] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'mavros_msgs/LogRequestListRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'start';
info.MatPath{2} = 'end';
