function [data, info] = set_named_digital_outputRequest
%set_named_digital_output gives an empty data for robotnik_msgs/set_named_digital_outputRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/set_named_digital_outputRequest';
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'robotnik_msgs/set_named_digital_outputRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'name';
info.MatPath{2} = 'value';
