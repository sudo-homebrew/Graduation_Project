function [data, info] = get_digital_inputResponse
%get_digital_input gives an empty data for robotnik_msgs/get_digital_inputResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/get_digital_inputResponse';
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'robotnik_msgs/get_digital_inputResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'value';
