function [data, info] = get_modbus_registerResponse
%get_modbus_register gives an empty data for robotnik_msgs/get_modbus_registerResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/get_modbus_registerResponse';
[data.Ret, info.Ret] = ros.internal.ros.messages.ros.default_type('logical',1);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('uint16',1);
info.MessageType = 'robotnik_msgs/get_modbus_registerResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,2);
info.MatPath{1} = 'ret';
info.MatPath{2} = 'value';
