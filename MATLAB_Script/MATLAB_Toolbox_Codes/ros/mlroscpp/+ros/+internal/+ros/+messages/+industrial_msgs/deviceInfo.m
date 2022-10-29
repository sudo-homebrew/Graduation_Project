function [data, info] = deviceInfo
%DeviceInfo gives an empty data for industrial_msgs/DeviceInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'industrial_msgs/DeviceInfo';
[data.Model, info.Model] = ros.internal.ros.messages.ros.char('string',0);
[data.SerialNumber, info.SerialNumber] = ros.internal.ros.messages.ros.char('string',0);
[data.HwVersion, info.HwVersion] = ros.internal.ros.messages.ros.char('string',0);
[data.SwVersion, info.SwVersion] = ros.internal.ros.messages.ros.char('string',0);
[data.Address, info.Address] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'industrial_msgs/DeviceInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'model';
info.MatPath{2} = 'serial_number';
info.MatPath{3} = 'hw_version';
info.MatPath{4} = 'sw_version';
info.MatPath{5} = 'address';
