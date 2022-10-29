function [data, info] = feature1D
%Feature1D gives an empty data for posedetection_msgs/Feature1D

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'posedetection_msgs/Feature1D';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Lines, info.Lines] = ros.internal.ros.messages.posedetection_msgs.curve1D;
info.Lines.MLdataType = 'struct';
info.Lines.MaxLen = NaN;
info.Lines.MinLen = 0;
data.Lines = data.Lines([],1);
[data.Descriptors, info.Descriptors] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.Confidences, info.Confidences] = ros.internal.ros.messages.ros.default_type('single',NaN);
[data.DescriptorDim, info.DescriptorDim] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'posedetection_msgs/Feature1D';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'lines';
info.MatPath{8} = 'lines.pts';
info.MatPath{9} = 'descriptors';
info.MatPath{10} = 'confidences';
info.MatPath{11} = 'descriptor_dim';
