function [data, info] = lookupTransformActionGoal
%LookupTransformActionGoal gives an empty data for tf2_msgs/LookupTransformActionGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'tf2_msgs/LookupTransformActionGoal';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.GoalId, info.GoalId] = ros.internal.ros.messages.actionlib_msgs.goalID;
info.GoalId.MLdataType = 'struct';
[data.Goal, info.Goal] = ros.internal.ros.messages.tf2_msgs.lookupTransformGoal;
info.Goal.MLdataType = 'struct';
info.MessageType = 'tf2_msgs/LookupTransformActionGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,25);
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
info.MatPath{13} = 'goal.target_frame';
info.MatPath{14} = 'goal.source_frame';
info.MatPath{15} = 'goal.source_time';
info.MatPath{16} = 'goal.source_time.sec';
info.MatPath{17} = 'goal.source_time.nsec';
info.MatPath{18} = 'goal.timeout';
info.MatPath{19} = 'goal.timeout.sec';
info.MatPath{20} = 'goal.timeout.nsec';
info.MatPath{21} = 'goal.target_time';
info.MatPath{22} = 'goal.target_time.sec';
info.MatPath{23} = 'goal.target_time.nsec';
info.MatPath{24} = 'goal.fixed_frame';
info.MatPath{25} = 'goal.advanced';