function [data, info] = set_float_valueRequest
%set_float_value gives an empty data for robotnik_msgs/set_float_valueRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/set_float_valueRequest';
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'robotnik_msgs/set_float_valueRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'value';
