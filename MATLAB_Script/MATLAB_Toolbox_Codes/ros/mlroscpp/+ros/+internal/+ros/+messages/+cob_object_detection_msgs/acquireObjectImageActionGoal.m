function [data, info] = acquireObjectImageActionGoal
%AcquireObjectImageActionGoal gives an empty data for cob_object_detection_msgs/AcquireObjectImageActionGoal

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_object_detection_msgs/AcquireObjectImageActionGoal';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.GoalId, info.GoalId] = ros.internal.ros.messages.actionlib_msgs.goalID;
info.GoalId.MLdataType = 'struct';
[data.Goal, info.Goal] = ros.internal.ros.messages.cob_object_detection_msgs.acquireObjectImageGoal;
info.Goal.MLdataType = 'struct';
info.MessageType = 'cob_object_detection_msgs/AcquireObjectImageActionGoal';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,48);
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
info.MatPath{14} = 'goal.reset_image_counter';
info.MatPath{15} = 'goal.pose';
info.MatPath{16} = 'goal.pose.header';
info.MatPath{17} = 'goal.pose.header.seq';
info.MatPath{18} = 'goal.pose.header.stamp';
info.MatPath{19} = 'goal.pose.header.stamp.sec';
info.MatPath{20} = 'goal.pose.header.stamp.nsec';
info.MatPath{21} = 'goal.pose.header.frame_id';
info.MatPath{22} = 'goal.pose.pose';
info.MatPath{23} = 'goal.pose.pose.position';
info.MatPath{24} = 'goal.pose.pose.position.x';
info.MatPath{25} = 'goal.pose.pose.position.y';
info.MatPath{26} = 'goal.pose.pose.position.z';
info.MatPath{27} = 'goal.pose.pose.orientation';
info.MatPath{28} = 'goal.pose.pose.orientation.x';
info.MatPath{29} = 'goal.pose.pose.orientation.y';
info.MatPath{30} = 'goal.pose.pose.orientation.z';
info.MatPath{31} = 'goal.pose.pose.orientation.w';
info.MatPath{32} = 'goal.sdh_joints';
info.MatPath{33} = 'goal.sdh_joints.header';
info.MatPath{34} = 'goal.sdh_joints.header.seq';
info.MatPath{35} = 'goal.sdh_joints.header.stamp';
info.MatPath{36} = 'goal.sdh_joints.header.stamp.sec';
info.MatPath{37} = 'goal.sdh_joints.header.stamp.nsec';
info.MatPath{38} = 'goal.sdh_joints.header.frame_id';
info.MatPath{39} = 'goal.sdh_joints.pose';
info.MatPath{40} = 'goal.sdh_joints.pose.position';
info.MatPath{41} = 'goal.sdh_joints.pose.position.x';
info.MatPath{42} = 'goal.sdh_joints.pose.position.y';
info.MatPath{43} = 'goal.sdh_joints.pose.position.z';
info.MatPath{44} = 'goal.sdh_joints.pose.orientation';
info.MatPath{45} = 'goal.sdh_joints.pose.orientation.x';
info.MatPath{46} = 'goal.sdh_joints.pose.orientation.y';
info.MatPath{47} = 'goal.sdh_joints.pose.orientation.z';
info.MatPath{48} = 'goal.sdh_joints.pose.orientation.w';
