function [data, info] = feature0D
%Feature0D gives an empty data for posedetection_msgs/Feature0D

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'posedetection_msgs/Feature0D';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Positions, info.Positions] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.Scales, info.Scales] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.Orientations, info.Orientations] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.Confidences, info.Confidences] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.Descriptors, info.Descriptors] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.DescriptorDim, info.DescriptorDim] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Type, info.Type] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'posedetection_msgs/Feature0D';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'positions';
info.MatPath{8} = 'scales';
info.MatPath{9} = 'orientations';
info.MatPath{10} = 'confidences';
info.MatPath{11} = 'descriptors';
info.MatPath{12} = 'descriptor_dim';
info.MatPath{13} = 'type';
