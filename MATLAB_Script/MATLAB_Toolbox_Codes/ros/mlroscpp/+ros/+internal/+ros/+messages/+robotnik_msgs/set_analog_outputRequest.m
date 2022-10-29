function [data, info] = set_analog_outputRequest
%set_analog_output gives an empty data for robotnik_msgs/set_analog_outputRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/set_analog_outputRequest';
[data.Output, info.Output] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'robotnik_msgs/set_analog_outputRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'output';
info.MatPath{2} = 'value';
