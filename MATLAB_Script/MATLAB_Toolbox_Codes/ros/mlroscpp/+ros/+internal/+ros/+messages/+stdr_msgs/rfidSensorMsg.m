function [data, info] = rfidSensorMsg
%RfidSensorMsg gives an empty data for stdr_msgs/RfidSensorMsg

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/RfidSensorMsg';
[data.MaxRange, info.MaxRange] = ros.internal.ros.messages.ros.default_type('single',1);
[data.AngleSpan, info.AngleSpan] = ros.internal.ros.messages.ros.default_type('single',1);
[data.SignalCutoff, info.SignalCutoff] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Frequency, info.Frequency] = ros.internal.ros.messages.ros.default_type('single',1);
[data.FrameId, info.FrameId] = ros.internal.ros.messages.ros.char('string',0);
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.Pose.MLdataType = 'struct';
info.MessageType = 'stdr_msgs/RfidSensorMsg';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'maxRange';
info.MatPath{2} = 'angleSpan';
info.MatPath{3} = 'signalCutoff';
info.MatPath{4} = 'frequency';
info.MatPath{5} = 'frame_id';
info.MatPath{6} = 'pose';
info.MatPath{7} = 'pose.x';
info.MatPath{8} = 'pose.y';
info.MatPath{9} = 'pose.theta';
