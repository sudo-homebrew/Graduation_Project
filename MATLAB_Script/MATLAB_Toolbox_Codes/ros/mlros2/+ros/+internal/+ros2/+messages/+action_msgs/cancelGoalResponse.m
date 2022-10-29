function [data, info] = cancelGoalResponse
%CancelGoal gives an empty data for action_msgs/CancelGoalResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'action_msgs/CancelGoalResponse';
[data.ERROR_NONE, info.ERROR_NONE] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 0, [NaN]);
[data.ERROR_REJECTED, info.ERROR_REJECTED] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 1, [NaN]);
[data.ERROR_UNKNOWN_GOAL_ID, info.ERROR_UNKNOWN_GOAL_ID] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 2, [NaN]);
[data.ERROR_GOAL_TERMINATED, info.ERROR_GOAL_TERMINATED] = ros.internal.ros2.messages.ros2.default_type('int8',1,0, 3, [NaN]);
[data.return_code, info.return_code] = ros.internal.ros2.messages.ros2.default_type('int8',1,0);
[data.goals_canceling, info.goals_canceling] = ros.internal.ros2.messages.action_msgs.goalInfo;
info.goals_canceling.MLdataType = 'struct';
info.goals_canceling.MaxLen = NaN;
info.goals_canceling.MinLen = 0;
info.MessageType = 'action_msgs/CancelGoalResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'ERROR_NONE';
info.MatPath{2} = 'ERROR_REJECTED';
info.MatPath{3} = 'ERROR_UNKNOWN_GOAL_ID';
info.MatPath{4} = 'ERROR_GOAL_TERMINATED';
info.MatPath{5} = 'return_code';
info.MatPath{6} = 'goals_canceling';
info.MatPath{7} = 'goals_canceling.goal_id';
info.MatPath{8} = 'goals_canceling.goal_id.uuid';
info.MatPath{9} = 'goals_canceling.stamp';
info.MatPath{10} = 'goals_canceling.stamp.sec';
info.MatPath{11} = 'goals_canceling.stamp.nanosec';
