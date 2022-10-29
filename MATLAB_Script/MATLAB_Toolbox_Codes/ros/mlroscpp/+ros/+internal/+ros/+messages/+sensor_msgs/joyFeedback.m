function [data, info] = joyFeedback
%JoyFeedback gives an empty data for sensor_msgs/JoyFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/JoyFeedback';
[data.TYPELED, info.TYPELED] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.TYPERUMBLE, info.TYPERUMBLE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.TYPEBUZZER, info.TYPEBUZZER] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.Type, info.Type] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Intensity, info.Intensity] = ros.internal.ros.messages.ros.default_type('single',1);
info.MessageType = 'sensor_msgs/JoyFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'TYPE_LED';
info.MatPath{2} = 'TYPE_RUMBLE';
info.MatPath{3} = 'TYPE_BUZZER';
info.MatPath{4} = 'type';
info.MatPath{5} = 'id';
info.MatPath{6} = 'intensity';
