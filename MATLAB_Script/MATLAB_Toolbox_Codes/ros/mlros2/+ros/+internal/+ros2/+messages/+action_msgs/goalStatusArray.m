function [data, info] = goalStatusArray
%GoalStatusArray gives an empty data for action_msgs/GoalStatusArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'action_msgs/GoalStatusArray';
[data.status_list, info.status_list] = ros.internal.ros2.messages.action_msgs.goalStatus;
info.status_list.MLdataType = 'struct';
info.status_list.MaxLen = NaN;
info.status_list.MinLen = 0;
info.MessageType = 'action_msgs/GoalStatusArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,15);
info.MatPath{1} = 'status_list';
info.MatPath{2} = 'status_list.STATUS_UNKNOWN';
info.MatPath{3} = 'status_list.STATUS_ACCEPTED';
info.MatPath{4} = 'status_list.STATUS_EXECUTING';
info.MatPath{5} = 'status_list.STATUS_CANCELING';
info.MatPath{6} = 'status_list.STATUS_SUCCEEDED';
info.MatPath{7} = 'status_list.STATUS_CANCELED';
info.MatPath{8} = 'status_list.STATUS_ABORTED';
info.MatPath{9} = 'status_list.goal_info';
info.MatPath{10} = 'status_list.goal_info.goal_id';
info.MatPath{11} = 'status_list.goal_info.goal_id.uuid';
info.MatPath{12} = 'status_list.goal_info.stamp';
info.MatPath{13} = 'status_list.goal_info.stamp.sec';
info.MatPath{14} = 'status_list.goal_info.stamp.nanosec';
info.MatPath{15} = 'status_list.status';
