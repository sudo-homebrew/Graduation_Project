function [data, info] = sound
%Sound gives an empty data for kobuki_msgs/Sound

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'kobuki_msgs/Sound';
[data.ON, info.ON] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.OFF, info.OFF] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.RECHARGE, info.RECHARGE] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.BUTTON, info.BUTTON] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.ERROR, info.ERROR] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.CLEANINGSTART, info.CLEANINGSTART] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.CLEANINGEND, info.CLEANINGEND] = ros.internal.ros.messages.ros.default_type('uint8',1, 6);
[data.Value, info.Value] = ros.internal.ros.messages.ros.default_type('uint8',1);
info.MessageType = 'kobuki_msgs/Sound';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'ON';
info.MatPath{2} = 'OFF';
info.MatPath{3} = 'RECHARGE';
info.MatPath{4} = 'BUTTON';
info.MatPath{5} = 'ERROR';
info.MatPath{6} = 'CLEANINGSTART';
info.MatPath{7} = 'CLEANINGEND';
info.MatPath{8} = 'value';
