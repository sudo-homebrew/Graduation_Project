function [data, info] = spawnModelRequest
%SpawnModel gives an empty data for gazebo_msgs/SpawnModelRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'gazebo_msgs/SpawnModelRequest';
[data.ModelName, info.ModelName] = ros.internal.ros.messages.ros.char('string',0);
[data.ModelXml, info.ModelXml] = ros.internal.ros.messages.ros.char('string',0);
[data.RobotNamespace, info.RobotNamespace] = ros.internal.ros.messages.ros.char('string',0);
[data.InitialPose, info.InitialPose] = ros.internal.ros.messages.geometry_msgs.pose;
info.InitialPose.MLdataType = 'struct';
[data.ReferenceFrame, info.ReferenceFrame] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'gazebo_msgs/SpawnModelRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'model_name';
info.MatPath{2} = 'model_xml';
info.MatPath{3} = 'robot_namespace';
info.MatPath{4} = 'initial_pose';
info.MatPath{5} = 'initial_pose.position';
info.MatPath{6} = 'initial_pose.position.x';
info.MatPath{7} = 'initial_pose.position.y';
info.MatPath{8} = 'initial_pose.position.z';
info.MatPath{9} = 'initial_pose.orientation';
info.MatPath{10} = 'initial_pose.orientation.x';
info.MatPath{11} = 'initial_pose.orientation.y';
info.MatPath{12} = 'initial_pose.orientation.z';
info.MatPath{13} = 'initial_pose.orientation.w';
info.MatPath{14} = 'reference_frame';
