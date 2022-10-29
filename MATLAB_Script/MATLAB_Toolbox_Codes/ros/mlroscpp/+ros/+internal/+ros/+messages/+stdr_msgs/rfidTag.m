function [data, info] = rfidTag
%RfidTag gives an empty data for stdr_msgs/RfidTag

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/RfidTag';
[data.TagId, info.TagId] = ros.internal.ros.messages.ros.char('string',0);
[data.Message, info.Message] = ros.internal.ros.messages.ros.char('string',0);
[data.Pose, info.Pose] = ros.internal.ros.messages.geometry_msgs.pose2D;
info.Pose.MLdataType = 'struct';
info.MessageType = 'stdr_msgs/RfidTag';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,6);
info.MatPath{1} = 'tag_id';
info.MatPath{2} = 'message';
info.MatPath{3} = 'pose';
info.MatPath{4} = 'pose.x';
info.MatPath{5} = 'pose.y';
info.MatPath{6} = 'pose.theta';
