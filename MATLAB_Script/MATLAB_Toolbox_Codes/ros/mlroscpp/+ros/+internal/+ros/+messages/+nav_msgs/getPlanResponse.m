function [data, info] = getPlanResponse
%GetPlan gives an empty data for nav_msgs/GetPlanResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav_msgs/GetPlanResponse';
[data.Plan, info.Plan] = ros.internal.ros.messages.nav_msgs.path;
info.Plan.MLdataType = 'struct';
info.MessageType = 'nav_msgs/GetPlanResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,24);
info.MatPath{1} = 'plan';
info.MatPath{2} = 'plan.header';
info.MatPath{3} = 'plan.header.seq';
info.MatPath{4} = 'plan.header.stamp';
info.MatPath{5} = 'plan.header.stamp.sec';
info.MatPath{6} = 'plan.header.stamp.nsec';
info.MatPath{7} = 'plan.header.frame_id';
info.MatPath{8} = 'plan.poses';
info.MatPath{9} = 'plan.poses.header';
info.MatPath{10} = 'plan.poses.header.seq';
info.MatPath{11} = 'plan.poses.header.stamp';
info.MatPath{12} = 'plan.poses.header.stamp.sec';
info.MatPath{13} = 'plan.poses.header.stamp.nsec';
info.MatPath{14} = 'plan.poses.header.frame_id';
info.MatPath{15} = 'plan.poses.pose';
info.MatPath{16} = 'plan.poses.pose.position';
info.MatPath{17} = 'plan.poses.pose.position.x';
info.MatPath{18} = 'plan.poses.pose.position.y';
info.MatPath{19} = 'plan.poses.pose.position.z';
info.MatPath{20} = 'plan.poses.pose.orientation';
info.MatPath{21} = 'plan.poses.pose.orientation.x';
info.MatPath{22} = 'plan.poses.pose.orientation.y';
info.MatPath{23} = 'plan.poses.pose.orientation.z';
info.MatPath{24} = 'plan.poses.pose.orientation.w';