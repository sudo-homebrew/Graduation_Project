function [data, info] = queryGraspsActionGoal
%QueryGraspsActionGoal gives an empty data for cob_grasp_generation/QueryGraspsActionGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_grasp_generation/QueryGraspsActionGoal';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.GoalId, info.GoalId] = ros.internal.ros.messages.actionlib_msgs.goalID;
info.GoalId.MLdataType = 'struct';
[data.Goal, info.Goal] = ros.internal.ros.messages.cob_grasp_generation.queryGraspsGoal;
info.Goal.MLdataType = 'struct';
info.MessageType = 'cob_grasp_generation/QueryGraspsActionGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,18);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'goal_id';
info.MatPath{8} = 'goal_id.stamp';
info.MatPath{9} = 'goal_id.stamp.sec';
info.MatPath{10} = 'goal_id.stamp.nsec';
info.MatPath{11} = 'goal_id.id';
info.MatPath{12} = 'goal';
info.MatPath{13} = 'goal.object_name';
info.MatPath{14} = 'goal.gripper_type';
info.MatPath{15} = 'goal.gripper_side';
info.MatPath{16} = 'goal.grasp_id';
info.MatPath{17} = 'goal.num_grasps';
info.MatPath{18} = 'goal.threshold';