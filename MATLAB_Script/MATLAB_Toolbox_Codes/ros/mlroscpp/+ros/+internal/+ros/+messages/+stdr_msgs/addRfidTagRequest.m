function [data, info] = addRfidTagRequest
%AddRfidTag gives an empty data for stdr_msgs/AddRfidTagRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/AddRfidTagRequest';
[data.NewTag, info.NewTag] = ros.internal.ros.messages.stdr_msgs.rfidTag;
info.NewTag.MLdataType = 'struct';
info.MessageType = 'stdr_msgs/AddRfidTagRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'newTag';
info.MatPath{2} = 'newTag.tag_id';
info.MatPath{3} = 'newTag.message';
info.MatPath{4} = 'newTag.pose';
info.MatPath{5} = 'newTag.pose.x';
info.MatPath{6} = 'newTag.pose.y';
info.MatPath{7} = 'newTag.pose.theta';
