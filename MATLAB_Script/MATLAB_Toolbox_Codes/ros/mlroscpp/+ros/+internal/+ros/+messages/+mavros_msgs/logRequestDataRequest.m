function [data, info] = logRequestDataRequest
%LogRequestData gives an empty data for mavros_msgs/LogRequestDataRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'mavros_msgs/LogRequestDataRequest';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('uint16',1);
[data.Offset, info.Offset] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Count, info.Count] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'mavros_msgs/LogRequestDataRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'id';
info.MatPath{2} = 'offset';
info.MatPath{3} = 'count';
