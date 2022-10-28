function [data, info] = externalPower
%ExternalPower gives an empty data for kobuki_msgs/ExternalPower

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/ExternalPower';
[data.PWR33V1A, info.PWR33V1A] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.PWR5V1A, info.PWR5V1A] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.PWR12V5A, info.PWR12V5A] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.PWR12V15A, info.PWR12V15A] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.OFF, info.OFF] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.ON, info.ON] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.Source, info.Source] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.State, info.State] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'kobuki_msgs/ExternalPower';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'PWR_3_3V1A';
info.MatPath{2} = 'PWR_5V1A';
info.MatPath{3} = 'PWR_12V5A';
info.MatPath{4} = 'PWR_12V1_5A';
info.MatPath{5} = 'OFF';
info.MatPath{6} = 'ON';
info.MatPath{7} = 'source';
info.MatPath{8} = 'state';
