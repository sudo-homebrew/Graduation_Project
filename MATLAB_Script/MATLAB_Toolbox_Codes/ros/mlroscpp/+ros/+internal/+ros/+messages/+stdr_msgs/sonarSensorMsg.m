function [data, info] = sonarSensorMsg
%SonarSensorMsg gives an empty data for stdr_msgs/SonarSensorMsg

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/SonarSensorMsg';
[data.MaxRange, info.MaxRange] = ros.internal.ros.messages.ros.default_type('single',1);
[data.MinRange, info.MinRange] = ros.internal.ros.messages.ros.default_type('single',1);
[data.ConeAngle, info.ConeAngle] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Frequency, info.Frequency] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Noise, info.Noise] = ros.internal.ros.messages.stdr_msgs.noise;
info.Noise.MLdataType = 'struct';
[data.FrameId, info.FrameId] = ros.internal.ros.messages.ros.char('string',0);
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.Pose.MLdataType = 'struct';
info.MessageType = 'stdr_msgs/SonarSensorMsg';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'maxRange';
info.MatPath{2} = 'minRange';
info.MatPath{3} = 'coneAngle';
info.MatPath{4} = 'frequency';
info.MatPath{5} = 'noise';
info.MatPath{6} = 'noise.noise';
info.MatPath{7} = 'noise.noiseMean';
info.MatPath{8} = 'noise.noiseStd';
info.MatPath{9} = 'frame_id';
info.MatPath{10} = 'pose';
info.MatPath{11} = 'pose.x';
info.MatPath{12} = 'pose.y';
info.MatPath{13} = 'pose.theta';
