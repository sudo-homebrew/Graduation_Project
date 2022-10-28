function [data, info] = joyFeedbackArray
%JoyFeedbackArray gives an empty data for sensor_msgs/JoyFeedbackArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/JoyFeedbackArray';
[data.array, info.array] = ros.internal.ros2.messages.sensor_msgs.joyFeedback;
info.array.MLdataType = 'struct';
info.array.MaxLen = NaN;
info.array.MinLen = 0;
info.MessageType = 'sensor_msgs/JoyFeedbackArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'array';
info.MatPath{2} = 'array.TYPE_LED';
info.MatPath{3} = 'array.TYPE_RUMBLE';
info.MatPath{4} = 'array.TYPE_BUZZER';
info.MatPath{5} = 'array.type';
info.MatPath{6} = 'array.id';
info.MatPath{7} = 'array.intensity';
