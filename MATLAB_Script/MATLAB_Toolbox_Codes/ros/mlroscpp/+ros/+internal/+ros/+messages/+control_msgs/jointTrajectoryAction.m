function [data, info] = jointTrajectoryAction
%JointTrajectoryAction gives an empty data for control_msgs/JointTrajectoryAction

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'control_msgs/JointTrajectoryAction';
[data.ActionGoal, info.ActionGoal] = ros.internal.ros.messages.control_msgs.jointTrajectoryActionGoal;
info.ActionGoal.MLdataType = 'struct';
[data.ActionResult, info.ActionResult] = ros.internal.ros.messages.control_msgs.jointTrajectoryActionResult;
info.ActionResult.MLdataType = 'struct';
[data.ActionFeedback, info.ActionFeedback] = ros.internal.ros.messages.control_msgs.jointTrajectoryActionFeedback;
info.ActionFeedback.MLdataType = 'struct';
info.MessageType = 'control_msgs/JointTrajectoryAction';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,81);
info.MatPath{1} = 'action_goal';
info.MatPath{2} = 'action_goal.header';
info.MatPath{3} = 'action_goal.header.seq';
info.MatPath{4} = 'action_goal.header.stamp';
info.MatPath{5} = 'action_goal.header.stamp.sec';
info.MatPath{6} = 'action_goal.header.stamp.nsec';
info.MatPath{7} = 'action_goal.header.frame_id';
info.MatPath{8} = 'action_goal.goal_id';
info.MatPath{9} = 'action_goal.goal_id.stamp';
info.MatPath{10} = 'action_goal.goal_id.stamp.sec';
info.MatPath{11} = 'action_goal.goal_id.stamp.nsec';
info.MatPath{12} = 'action_goal.goal_id.id';
info.MatPath{13} = 'action_goal.goal';
info.MatPath{14} = 'action_goal.goal.trajectory';
info.MatPath{15} = 'action_goal.goal.trajectory.header';
info.MatPath{16} = 'action_goal.goal.trajectory.header.seq';
info.MatPath{17} = 'action_goal.goal.trajectory.header.stamp';
info.MatPath{18} = 'action_goal.goal.trajectory.header.stamp.sec';
info.MatPath{19} = 'action_goal.goal.trajectory.header.stamp.nsec';
info.MatPath{20} = 'action_goal.goal.trajectory.header.frame_id';
info.MatPath{21} = 'action_goal.goal.trajectory.joint_names';
info.MatPath{22} = 'action_goal.goal.trajectory.points';
info.MatPath{23} = 'action_goal.goal.trajectory.points.positions';
info.MatPath{24} = 'action_goal.goal.trajectory.points.velocities';
info.MatPath{25} = 'action_goal.goal.trajectory.points.accelerations';
info.MatPath{26} = 'action_goal.goal.trajectory.points.effort';
info.MatPath{27} = 'action_goal.goal.trajectory.points.time_from_start';
info.MatPath{28} = 'action_goal.goal.trajectory.points.time_from_start.sec';
info.MatPath{29} = 'action_goal.goal.trajectory.points.time_from_start.nsec';
info.MatPath{30} = 'action_result';
info.MatPath{31} = 'action_result.header';
info.MatPath{32} = 'action_result.header.seq';
info.MatPath{33} = 'action_result.header.stamp';
info.MatPath{34} = 'action_result.header.stamp.sec';
info.MatPath{35} = 'action_result.header.stamp.nsec';
info.MatPath{36} = 'action_result.header.frame_id';
info.MatPath{37} = 'action_result.status';
info.MatPath{38} = 'action_result.status.goal_id';
info.MatPath{39} = 'action_result.status.goal_id.stamp';
info.MatPath{40} = 'action_result.status.goal_id.stamp.sec';
info.MatPath{41} = 'action_result.status.goal_id.stamp.nsec';
info.MatPath{42} = 'action_result.status.goal_id.id';
info.MatPath{43} = 'action_result.status.status';
info.MatPath{44} = 'action_result.status.PENDING';
info.MatPath{45} = 'action_result.status.ACTIVE';
info.MatPath{46} = 'action_result.status.PREEMPTED';
info.MatPath{47} = 'action_result.status.SUCCEEDED';
info.MatPath{48} = 'action_result.status.ABORTED';
info.MatPath{49} = 'action_result.status.REJECTED';
info.MatPath{50} = 'action_result.status.PREEMPTING';
info.MatPath{51} = 'action_result.status.RECALLING';
info.MatPath{52} = 'action_result.status.RECALLED';
info.MatPath{53} = 'action_result.status.LOST';
info.MatPath{54} = 'action_result.status.text';
info.MatPath{55} = 'action_result.result';
info.MatPath{56} = 'action_feedback';
info.MatPath{57} = 'action_feedback.header';
info.MatPath{58} = 'action_feedback.header.seq';
info.MatPath{59} = 'action_feedback.header.stamp';
info.MatPath{60} = 'action_feedback.header.stamp.sec';
info.MatPath{61} = 'action_feedback.header.stamp.nsec';
info.MatPath{62} = 'action_feedback.header.frame_id';
info.MatPath{63} = 'action_feedback.status';
info.MatPath{64} = 'action_feedback.status.goal_id';
info.MatPath{65} = 'action_feedback.status.goal_id.stamp';
info.MatPath{66} = 'action_feedback.status.goal_id.stamp.sec';
info.MatPath{67} = 'action_feedback.status.goal_id.stamp.nsec';
info.MatPath{68} = 'action_feedback.status.goal_id.id';
info.MatPath{69} = 'action_feedback.status.status';
info.MatPath{70} = 'action_feedback.status.PENDING';
info.MatPath{71} = 'action_feedback.status.ACTIVE';
info.MatPath{72} = 'action_feedback.status.PREEMPTED';
info.MatPath{73} = 'action_feedback.status.SUCCEEDED';
info.MatPath{74} = 'action_feedback.status.ABORTED';
info.MatPath{75} = 'action_feedback.status.REJECTED';
info.MatPath{76} = 'action_feedback.status.PREEMPTING';
info.MatPath{77} = 'action_feedback.status.RECALLING';
info.MatPath{78} = 'action_feedback.status.RECALLED';
info.MatPath{79} = 'action_feedback.status.LOST';
info.MatPath{80} = 'action_feedback.status.text';
info.MatPath{81} = 'action_feedback.feedback';