function [data, info] = resetMappingRequest
%ResetMapping gives an empty data for hector_mapping/ResetMappingRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_mapping/ResetMappingRequest';
[data.InitialPose, info.InitialPose] = ros.internal.ros.messages.geometry_msgs.pose;
info.InitialPose.MLdataType = 'struct';
info.MessageType = 'hector_mapping/ResetMappingRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'initial_pose';
info.MatPath{2} = 'initial_pose.position';
info.MatPath{3} = 'initial_pose.position.x';
info.MatPath{4} = 'initial_pose.position.y';
info.MatPath{5} = 'initial_pose.position.z';
info.MatPath{6} = 'initial_pose.orientation';
info.MatPath{7} = 'initial_pose.orientation.x';
info.MatPath{8} = 'initial_pose.orientation.y';
info.MatPath{9} = 'initial_pose.orientation.z';
info.MatPath{10} = 'initial_pose.orientation.w';
