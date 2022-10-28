function [data, info] = goalInfo
%GoalInfo gives an empty data for action_msgs/GoalInfo

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'action_msgs/GoalInfo';
[data.goal_id, info.goal_id] = ros.internal.ros2.messages.unique_identifier_msgs.uUID;
info.goal_id.MLdataType = 'struct';
[data.stamp, info.stamp] = ros.internal.ros2.messages.builtin_interfaces.time;
info.stamp.MLdataType = 'struct';
info.MessageType = 'action_msgs/GoalInfo';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,5);
info.MatPath{1} = 'goal_id';
info.MatPath{2} = 'goal_id.uuid';
info.MatPath{3} = 'stamp';
info.MatPath{4} = 'stamp.sec';
info.MatPath{5} = 'stamp.nanosec';
