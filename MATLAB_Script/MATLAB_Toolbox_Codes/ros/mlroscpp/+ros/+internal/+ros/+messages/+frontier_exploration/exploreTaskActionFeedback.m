function [data, info] = exploreTaskActionFeedback
%ExploreTaskActionFeedback gives an empty data for frontier_exploration/ExploreTaskActionFeedback

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'frontier_exploration/ExploreTaskActionFeedback';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Status, info.Status] = ros.internal.ros.messages.actionlib_msgs.goalStatus;
info.Status.MLdataType = 'struct';
[data.Feedback, info.Feedback] = ros.internal.ros.messages.frontier_exploration.exploreTaskFeedback;
info.Feedback.MLdataType = 'struct';
info.MessageType = 'frontier_exploration/ExploreTaskActionFeedback';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,59);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'status';
info.MatPath{8} = 'status.goal_id';
info.MatPath{9} = 'status.goal_id.stamp';
info.MatPath{10} = 'status.goal_id.stamp.sec';
info.MatPath{11} = 'status.goal_id.stamp.nsec';
info.MatPath{12} = 'status.goal_id.id';
info.MatPath{13} = 'status.status';
info.MatPath{14} = 'status.PENDING';
info.MatPath{15} = 'status.ACTIVE';
info.MatPath{16} = 'status.PREEMPTED';
info.MatPath{17} = 'status.SUCCEEDED';
info.MatPath{18} = 'status.ABORTED';
info.MatPath{19} = 'status.REJECTED';
info.MatPath{20} = 'status.PREEMPTING';
info.MatPath{21} = 'status.RECALLING';
info.MatPath{22} = 'status.RECALLED';
info.MatPath{23} = 'status.LOST';
info.MatPath{24} = 'status.text';
info.MatPath{25} = 'feedback';
info.MatPath{26} = 'feedback.next_frontier';
info.MatPath{27} = 'feedback.next_frontier.header';
info.MatPath{28} = 'feedback.next_frontier.header.seq';
info.MatPath{29} = 'feedback.next_frontier.header.stamp';
info.MatPath{30} = 'feedback.next_frontier.header.stamp.sec';
info.MatPath{31} = 'feedback.next_frontier.header.stamp.nsec';
info.MatPath{32} = 'feedback.next_frontier.header.frame_id';
info.MatPath{33} = 'feedback.next_frontier.pose';
info.MatPath{34} = 'feedback.next_frontier.pose.position';
info.MatPath{35} = 'feedback.next_frontier.pose.position.x';
info.MatPath{36} = 'feedback.next_frontier.pose.position.y';
info.MatPath{37} = 'feedback.next_frontier.pose.position.z';
info.MatPath{38} = 'feedback.next_frontier.pose.orientation';
info.MatPath{39} = 'feedback.next_frontier.pose.orientation.x';
info.MatPath{40} = 'feedback.next_frontier.pose.orientation.y';
info.MatPath{41} = 'feedback.next_frontier.pose.orientation.z';
info.MatPath{42} = 'feedback.next_frontier.pose.orientation.w';
info.MatPath{43} = 'feedback.base_position';
info.MatPath{44} = 'feedback.base_position.header';
info.MatPath{45} = 'feedback.base_position.header.seq';
info.MatPath{46} = 'feedback.base_position.header.stamp';
info.MatPath{47} = 'feedback.base_position.header.stamp.sec';
info.MatPath{48} = 'feedback.base_position.header.stamp.nsec';
info.MatPath{49} = 'feedback.base_position.header.frame_id';
info.MatPath{50} = 'feedback.base_position.pose';
info.MatPath{51} = 'feedback.base_position.pose.position';
info.MatPath{52} = 'feedback.base_position.pose.position.x';
info.MatPath{53} = 'feedback.base_position.pose.position.y';
info.MatPath{54} = 'feedback.base_position.pose.position.z';
info.MatPath{55} = 'feedback.base_position.pose.orientation';
info.MatPath{56} = 'feedback.base_position.pose.orientation.x';
info.MatPath{57} = 'feedback.base_position.pose.orientation.y';
info.MatPath{58} = 'feedback.base_position.pose.orientation.z';
info.MatPath{59} = 'feedback.base_position.pose.orientation.w';