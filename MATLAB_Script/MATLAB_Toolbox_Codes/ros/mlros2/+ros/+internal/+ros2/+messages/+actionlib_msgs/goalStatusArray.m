function [data, info] = goalStatusArray
%GoalStatusArray gives an empty data for actionlib_msgs/GoalStatusArray

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'actionlib_msgs/GoalStatusArray';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.status_list, info.status_list] = ros.internal.ros2.messages.actionlib_msgs.goalStatus;
info.status_list.MLdataType = 'struct';
info.status_list.MaxLen = NaN;
info.status_list.MinLen = 0;
info.MessageType = 'actionlib_msgs/GoalStatusArray';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,23);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'status_list';
info.MatPath{7} = 'status_list.goal_id';
info.MatPath{8} = 'status_list.goal_id.stamp';
info.MatPath{9} = 'status_list.goal_id.stamp.sec';
info.MatPath{10} = 'status_list.goal_id.stamp.nanosec';
info.MatPath{11} = 'status_list.goal_id.id';
info.MatPath{12} = 'status_list.status';
info.MatPath{13} = 'status_list.PENDING';
info.MatPath{14} = 'status_list.ACTIVE';
info.MatPath{15} = 'status_list.PREEMPTED';
info.MatPath{16} = 'status_list.SUCCEEDED';
info.MatPath{17} = 'status_list.ABORTED';
info.MatPath{18} = 'status_list.REJECTED';
info.MatPath{19} = 'status_list.PREEMPTING';
info.MatPath{20} = 'status_list.RECALLING';
info.MatPath{21} = 'status_list.RECALLED';
info.MatPath{22} = 'status_list.LOST';
info.MatPath{23} = 'status_list.text';
