function [data, info] = cancelGoalRequest
%CancelGoal gives an empty data for action_msgs/CancelGoalRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'action_msgs/CancelGoalRequest';
[data.goal_info, info.goal_info] = ros.internal.ros2.messages.action_msgs.goalInfo;
info.goal_info.MLdataType = 'struct';
info.MessageType = 'action_msgs/CancelGoalRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'goal_info';
info.MatPath{2} = 'goal_info.goal_id';
info.MatPath{3} = 'goal_info.goal_id.uuid';
info.MatPath{4} = 'goal_info.stamp';
info.MatPath{5} = 'goal_info.stamp.sec';
info.MatPath{6} = 'goal_info.stamp.nanosec';
