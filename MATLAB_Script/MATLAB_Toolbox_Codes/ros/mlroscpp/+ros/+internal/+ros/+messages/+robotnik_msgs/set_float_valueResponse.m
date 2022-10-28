function [data, info] = set_float_valueResponse
%set_float_value gives an empty data for robotnik_msgs/set_float_valueResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/set_float_valueResponse';
[data.Ret, info.Ret] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.ErrorMessage, info.ErrorMessage] = ros.internal.ros.messages.std_msgs.string;
info.ErrorMessage.MLdataType = 'struct';
info.MessageType = 'robotnik_msgs/set_float_valueResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'ret';
info.MatPath{2} = 'errorMessage';
info.MatPath{3} = 'errorMessage.data';
