function [data, info] = task
%Task gives an empty data for data_vis_msgs/Task

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'data_vis_msgs/Task';
[data.Id, info.Id] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Parent, info.Parent] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Color, info.Color] = ros.internal.ros.messages.ros.char('string',0);
[data.Info, info.Info] = ros.internal.ros.messages.ros.char('string',0);
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'data_vis_msgs/Task';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'id';
info.MatPath{2} = 'parent';
info.MatPath{3} = 'color';
info.MatPath{4} = 'info';
info.MatPath{5} = 'type';
