function [data, info] = get_modbus_registerRequest
%get_modbus_register gives an empty data for robotnik_msgs/get_modbus_registerRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'robotnik_msgs/get_modbus_registerRequest';
[data.Address, info.Address] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'robotnik_msgs/get_modbus_registerRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'address';
