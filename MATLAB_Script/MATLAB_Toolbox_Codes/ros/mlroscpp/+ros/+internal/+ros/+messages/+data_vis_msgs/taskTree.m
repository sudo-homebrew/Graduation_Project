function [data, info] = taskTree
%TaskTree gives an empty data for data_vis_msgs/TaskTree

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'data_vis_msgs/TaskTree';
[data.Height, info.Height] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Tree, info.Tree] = ros.internal.ros.messages.data_vis_msgs.task;
info.Tree.MLdataType = 'struct';
info.Tree.MaxLen = NaN;
info.Tree.MinLen = 0;
data.Tree = data.Tree([],1);
info.MessageType = 'data_vis_msgs/TaskTree';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'height';
info.MatPath{2} = 'width';
info.MatPath{3} = 'tree';
info.MatPath{4} = 'tree.id';
info.MatPath{5} = 'tree.parent';
info.MatPath{6} = 'tree.color';
info.MatPath{7} = 'tree.info';
info.MatPath{8} = 'tree.type';
