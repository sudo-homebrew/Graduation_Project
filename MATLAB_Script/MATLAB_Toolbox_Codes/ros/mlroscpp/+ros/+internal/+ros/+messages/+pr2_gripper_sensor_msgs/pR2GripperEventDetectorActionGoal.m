function [data, info] = pR2GripperEventDetectorActionGoal
%PR2GripperEventDetectorActionGoal gives an empty data for pr2_gripper_sensor_msgs/PR2GripperEventDetectorActionGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperEventDetectorActionGoal';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.GoalId, info.GoalId] = ros.internal.ros.messages.actionlib_msgs.goalID;
info.GoalId.MLdataType = 'struct';
[data.Goal, info.Goal] = ros.internal.ros.messages.pr2_gripper_sensor_msgs.pR2GripperEventDetectorGoal;
info.Goal.MLdataType = 'struct';
info.MessageType = 'pr2_gripper_sensor_msgs/PR2GripperEventDetectorActionGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,21);
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
info.MatPath{13} = 'goal.command';
info.MatPath{14} = 'goal.command.trigger_conditions';
info.MatPath{15} = 'goal.command.FINGER_SIDE_IMPACT_OR_ACC';
info.MatPath{16} = 'goal.command.SLIP_AND_ACC';
info.MatPath{17} = 'goal.command.FINGER_SIDE_IMPACT_OR_SLIP_OR_ACC';
info.MatPath{18} = 'goal.command.SLIP';
info.MatPath{19} = 'goal.command.ACC';
info.MatPath{20} = 'goal.command.acceleration_trigger_magnitude';
info.MatPath{21} = 'goal.command.slip_trigger_magnitude';