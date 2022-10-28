function [data, info] = register
%Register gives an empty data for robotnik_msgs/Register

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/Register';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'robotnik_msgs/Register';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'id';
info.MatPath{2} = 'value';
