function [data, info] = cO2SensorMsg
%CO2SensorMsg gives an empty data for stdr_msgs/CO2SensorMsg

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/CO2SensorMsg';
[data.MaxRange, info.MaxRange] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Frequency, info.Frequency] = ros.internal.ros.messages.ros.default_type('single',1);
[data.FrameId, info.FrameId] = ros.internal.ros.messages.ros.char('string',0);
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.Pose.MLdataType = 'struct';
info.MessageType = 'stdr_msgs/CO2SensorMsg';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'maxRange';
info.MatPath{2} = 'frequency';
info.MatPath{3} = 'frame_id';
info.MatPath{4} = 'pose';
info.MatPath{5} = 'pose.x';
info.MatPath{6} = 'pose.y';
info.MatPath{7} = 'pose.theta';
