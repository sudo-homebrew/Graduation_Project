function [data, info] = speech
%Speech gives an empty data for data_vis_msgs/Speech

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'data_vis_msgs/Speech';
[data.Id, info.Id] = ros.internal.ros.messages.ros.char('string',0);
[data.Text, info.Text] = ros.internal.ros.messages.ros.char('string',0);
[data.Position, info.Position] = ros.internal.ros.messages.geometry_msgs.point;
info.Position.MLdataType = 'struct';
[data.Duration, info.Duration] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'data_vis_msgs/Speech';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'id';
info.MatPath{2} = 'text';
info.MatPath{3} = 'position';
info.MatPath{4} = 'position.x';
info.MatPath{5} = 'position.y';
info.MatPath{6} = 'position.z';
info.MatPath{7} = 'duration';
