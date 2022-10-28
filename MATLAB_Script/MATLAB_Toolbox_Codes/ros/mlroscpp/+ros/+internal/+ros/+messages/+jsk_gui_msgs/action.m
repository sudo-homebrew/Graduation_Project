function [data, info] = action
%Action gives an empty data for jsk_gui_msgs/Action

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_gui_msgs/Action';
[data.RARMID, info.RARMID] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.LARMID, info.LARMID] = ros.internal.ros.messages.ros.default_type('int8',1, 1);
[data.TaskName, info.TaskName] = ros.internal.ros.messages.ros.char('string',0);
[data.ArmId, info.ArmId] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.State, info.State] = ros.internal.ros.messages.ros.char('string',0);
[data.StateValue, info.StateValue] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Direction, info.Direction] = ros.internal.ros.messages.ros.char('string',0);
[data.DirectionValue, info.DirectionValue] = ros.internal.ros.messages.ros.default_type('double',1);
[data.TouchX, info.TouchX] = ros.internal.ros.messages.ros.default_type('int64',1);
[data.TouchY, info.TouchY] = ros.internal.ros.messages.ros.default_type('int64',1);
info.MessageType = 'jsk_gui_msgs/Action';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'RARMID';
info.MatPath{2} = 'LARMID';
info.MatPath{3} = 'task_name';
info.MatPath{4} = 'arm_id';
info.MatPath{5} = 'state';
info.MatPath{6} = 'state_value';
info.MatPath{7} = 'direction';
info.MatPath{8} = 'direction_value';
info.MatPath{9} = 'touch_x';
info.MatPath{10} = 'touch_y';
