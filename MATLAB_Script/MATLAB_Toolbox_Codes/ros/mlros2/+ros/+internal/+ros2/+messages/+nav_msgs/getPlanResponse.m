function [data, info] = getPlanResponse
%GetPlan gives an empty data for nav_msgs/GetPlanResponse

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav_msgs/GetPlanResponse';
[data.plan, info.plan] = ros.internal.ros2.messages.nav_msgs.path;
info.plan.MLdataType = 'struct';
info.MessageType = 'nav_msgs/GetPlanResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,22);
info.MatPath{1} = 'plan';
info.MatPath{2} = 'plan.header';
info.MatPath{3} = 'plan.header.stamp';
info.MatPath{4} = 'plan.header.stamp.sec';
info.MatPath{5} = 'plan.header.stamp.nanosec';
info.MatPath{6} = 'plan.header.frame_id';
info.MatPath{7} = 'plan.poses';
info.MatPath{8} = 'plan.poses.header';
info.MatPath{9} = 'plan.poses.header.stamp';
info.MatPath{10} = 'plan.poses.header.stamp.sec';
info.MatPath{11} = 'plan.poses.header.stamp.nanosec';
info.MatPath{12} = 'plan.poses.header.frame_id';
info.MatPath{13} = 'plan.poses.pose';
info.MatPath{14} = 'plan.poses.pose.position';
info.MatPath{15} = 'plan.poses.pose.position.x';
info.MatPath{16} = 'plan.poses.pose.position.y';
info.MatPath{17} = 'plan.poses.pose.position.z';
info.MatPath{18} = 'plan.poses.pose.orientation';
info.MatPath{19} = 'plan.poses.pose.orientation.x';
info.MatPath{20} = 'plan.poses.pose.orientation.y';
info.MatPath{21} = 'plan.poses.pose.orientation.z';
info.MatPath{22} = 'plan.poses.pose.orientation.w';
