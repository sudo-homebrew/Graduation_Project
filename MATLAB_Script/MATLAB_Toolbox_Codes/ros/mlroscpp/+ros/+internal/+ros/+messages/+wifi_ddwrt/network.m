function [data, info] = network
%Network gives an empty data for wifi_ddwrt/Network

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'wifi_ddwrt/Network';
[data.Macattr, info.Macattr] = ros.internal.ros.messages.ros.char('string',0);
[data.Essid, info.Essid] = ros.internal.ros.messages.ros.char('string',0);
[data.Channel, info.Channel] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Rssi, info.Rssi] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Noise, info.Noise] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Beacon, info.Beacon] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'wifi_ddwrt/Network';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'macattr';
info.MatPath{2} = 'essid';
info.MatPath{3} = 'channel';
info.MatPath{4} = 'rssi';
info.MatPath{5} = 'noise';
info.MatPath{6} = 'beacon';
