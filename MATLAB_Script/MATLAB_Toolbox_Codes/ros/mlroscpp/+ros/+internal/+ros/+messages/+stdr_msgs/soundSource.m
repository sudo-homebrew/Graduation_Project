function [data, info] = soundSource
%SoundSource gives an empty data for stdr_msgs/SoundSource

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/SoundSource';
[data.Id, info.Id] = ros.internal.ros.messages.ros.char('string',0);
[data.Dbs, info.Dbs] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.Pose.MLdataType = 'struct';
info.MessageType = 'stdr_msgs/SoundSource';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'id';
info.MatPath{2} = 'dbs';
info.MatPath{3} = 'pose';
info.MatPath{4} = 'pose.x';
info.MatPath{5} = 'pose.y';
info.MatPath{6} = 'pose.theta';