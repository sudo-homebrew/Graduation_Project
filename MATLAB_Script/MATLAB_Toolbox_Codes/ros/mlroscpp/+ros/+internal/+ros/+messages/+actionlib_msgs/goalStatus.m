function [data, info] = goalStatus
%GoalStatus gives an empty data for actionlib_msgs/GoalStatus

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'actionlib_msgs/GoalStatus';
[data.GoalId, info.GoalId] = ros.internal.ros.messages.actionlib_msgs.goalID;
info.GoalId.MLdataType = 'struct';
[data.Status, info.Status] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.PENDING, info.PENDING] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.ACTIVE, info.ACTIVE] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.PREEMPTED, info.PREEMPTED] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.SUCCEEDED, info.SUCCEEDED] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.ABORTED, info.ABORTED] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.REJECTED, info.REJECTED] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.PREEMPTING, info.PREEMPTING] = ros.internal.ros.messages.ros.default_type('uint8',1, 6);
[data.RECALLING, info.RECALLING] = ros.internal.ros.messages.ros.default_type('uint8',1, 7);
[data.RECALLED, info.RECALLED] = ros.internal.ros.messages.ros.default_type('uint8',1, 8);
[data.LOST, info.LOST] = ros.internal.ros.messages.ros.default_type('uint8',1, 9);
[data.Text, info.Text] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'actionlib_msgs/GoalStatus';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'goal_id';
info.MatPath{2} = 'goal_id.stamp';
info.MatPath{3} = 'goal_id.stamp.sec';
info.MatPath{4} = 'goal_id.stamp.nsec';
info.MatPath{5} = 'goal_id.id';
info.MatPath{6} = 'status';
info.MatPath{7} = 'PENDING';
info.MatPath{8} = 'ACTIVE';
info.MatPath{9} = 'PREEMPTED';
info.MatPath{10} = 'SUCCEEDED';
info.MatPath{11} = 'ABORTED';
info.MatPath{12} = 'REJECTED';
info.MatPath{13} = 'PREEMPTING';
info.MatPath{14} = 'RECALLING';
info.MatPath{15} = 'RECALLED';
info.MatPath{16} = 'LOST';
info.MatPath{17} = 'text';