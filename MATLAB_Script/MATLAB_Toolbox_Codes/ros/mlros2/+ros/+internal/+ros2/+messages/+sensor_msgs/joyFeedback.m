function [data, info] = joyFeedback
%JoyFeedback gives an empty data for sensor_msgs/JoyFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/JoyFeedback';
[data.TYPE_LED, info.TYPE_LED] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 0, [NaN]);
[data.TYPE_RUMBLE, info.TYPE_RUMBLE] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 1, [NaN]);
[data.TYPE_BUZZER, info.TYPE_BUZZER] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0, 2, [NaN]);
[data.type, info.type] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.id, info.id] = ros.internal.ros2.messages.ros2.default_type('uint8',1,0);
[data.intensity, info.intensity] = ros.internal.ros2.messages.ros2.default_type('single',1,0);
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
