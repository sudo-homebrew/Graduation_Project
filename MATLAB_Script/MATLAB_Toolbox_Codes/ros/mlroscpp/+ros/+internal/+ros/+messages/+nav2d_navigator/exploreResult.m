function [data, info] = exploreResult
%ExploreResult gives an empty data for nav2d_navigator/ExploreResult

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'nav2d_navigator/ExploreResult';
[data.FinalPose, info.FinalPose] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.FinalPose.MLdataType = 'struct';
info.MessageType = 'nav2d_navigator/ExploreResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,4);
info.MatPath{1} = 'final_pose';
info.MatPath{2} = 'final_pose.x';
info.MatPath{3} = 'final_pose.y';
info.MatPath{4} = 'final_pose.theta';
