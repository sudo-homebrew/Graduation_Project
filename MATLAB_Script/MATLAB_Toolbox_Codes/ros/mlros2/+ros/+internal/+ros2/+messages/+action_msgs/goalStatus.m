function [data, info] = goalStatus
%GoalStatus gives an empty data for action_msgs/GoalStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'action_msgs/GoalStatus';
[data.STATUS_UNKNOWN, info.STATUS_UNKNOWN] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 0, [NaN]);
[data.STATUS_ACCEPTED, info.STATUS_ACCEPTED] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 1, [NaN]);
[data.STATUS_EXECUTING, info.STATUS_EXECUTING] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 2, [NaN]);
[data.STATUS_CANCELING, info.STATUS_CANCELING] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 3, [NaN]);
[data.STATUS_SUCCEEDED, info.STATUS_SUCCEEDED] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 4, [NaN]);
[data.STATUS_CANCELED, info.STATUS_CANCELED] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 5, [NaN]);
[data.STATUS_ABORTED, info.STATUS_ABORTED] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 6, [NaN]);
[data.goal_info, info.goal_info] = ros.internal.ros2.messages.action_msgs.goalInfo;
info.goal_info.MLdataType = 'struct';
[data.status, info.status] = ros.internal.ros2.messages.ros2.default_type('int8',1,0);
info.MessageType = 'action_msgs/GoalStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'STATUS_UNKNOWN';
info.MatPath{2} = 'STATUS_ACCEPTED';
info.MatPath{3} = 'STATUS_EXECUTING';
info.MatPath{4} = 'STATUS_CANCELING';
info.MatPath{5} = 'STATUS_SUCCEEDED';
info.MatPath{6} = 'STATUS_CANCELED';
info.MatPath{7} = 'STATUS_ABORTED';
info.MatPath{8} = 'goal_info';
info.MatPath{9} = 'goal_info.goal_id';
info.MatPath{10} = 'goal_info.goal_id.uuid';
info.MatPath{11} = 'goal_info.stamp';
info.MatPath{12} = 'goal_info.stamp.sec';
info.MatPath{13} = 'goal_info.stamp.nanosec';
info.MatPath{14} = 'status';
