function [data, info] = network
%Network gives an empty data for wireless_msgs/Network

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'wireless_msgs/Network';
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
[data.Essid, info.Essid] = ros.internal.ros.messages.ros.char('string',0);
[data.Mac, info.Mac] = ros.internal.ros.messages.ros.char('string',0);
[data.Mode, info.Mode] = ros.internal.ros.messages.ros.char('string',0);
[data.Frequency, info.Frequency] = ros.internal.ros.messages.ros.char('string',0);
[data.Encryption, info.Encryption] = ros.internal.ros.messages.ros.default_type('logical',1);
info.MessageType = 'wireless_msgs/Network';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'type';
info.MatPath{2} = 'essid';
info.MatPath{3} = 'mac';
info.MatPath{4} = 'mode';
info.MatPath{5} = 'frequency';
info.MatPath{6} = 'encryption';
