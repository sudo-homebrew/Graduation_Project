function [data, info] = queryRequest
%Query gives an empty data for jsk_gui_msgs/QueryRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_gui_msgs/QueryRequest';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'jsk_gui_msgs/QueryRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'id';
