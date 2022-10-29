function [data, info] = objectColor
%ObjectColor gives an empty data for moveit_msgs/ObjectColor

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/ObjectColor';
[data.Id, info.Id] = ros.internal.ros.messages.ros.char('string',0);
[data.Color, info.Color] = ros.internal.ros.messages.std_msgs.colorRGBA;
info.Color.MLdataType = 'struct';
info.MessageType = 'moveit_msgs/ObjectColor';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'id';
info.MatPath{2} = 'color';
info.MatPath{3} = 'color.r';
info.MatPath{4} = 'color.g';
info.MatPath{5} = 'color.b';
info.MatPath{6} = 'color.a';
