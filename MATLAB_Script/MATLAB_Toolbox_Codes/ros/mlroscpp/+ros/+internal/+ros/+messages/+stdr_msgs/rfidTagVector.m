function [data, info] = rfidTagVector
%RfidTagVector gives an empty data for stdr_msgs/RfidTagVector

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'stdr_msgs/RfidTagVector';
[data.RfidTags, info.RfidTags] = ros.internal.ros.messages.stdr_msgs.rfidTag;
info.RfidTags.MLdataType = 'struct';
info.RfidTags.MaxLen = NaN;
info.RfidTags.MinLen = 0;
data.RfidTags = data.RfidTags([],1);
info.MessageType = 'stdr_msgs/RfidTagVector';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,7);
info.MatPath{1} = 'rfid_tags';
info.MatPath{2} = 'rfid_tags.tag_id';
info.MatPath{3} = 'rfid_tags.message';
info.MatPath{4} = 'rfid_tags.pose';
info.MatPath{5} = 'rfid_tags.pose.x';
info.MatPath{6} = 'rfid_tags.pose.y';
info.MatPath{7} = 'rfid_tags.pose.theta';
