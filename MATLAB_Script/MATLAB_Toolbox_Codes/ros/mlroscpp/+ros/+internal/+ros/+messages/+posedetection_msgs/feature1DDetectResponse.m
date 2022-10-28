function [data, info] = feature1DDetectResponse
%Feature1DDetect gives an empty data for posedetection_msgs/Feature1DDetectResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'posedetection_msgs/Feature1DDetectResponse';
[data.Features, info.Features] = ros.internal.ros.messages.posedetection_msgs.feature1D;
info.Features.MLdataType = 'struct';
info.MessageType = 'posedetection_msgs/Feature1DDetectResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'features';
info.MatPath{2} = 'features.header';
info.MatPath{3} = 'features.header.seq';
info.MatPath{4} = 'features.header.stamp';
info.MatPath{5} = 'features.header.stamp.sec';
info.MatPath{6} = 'features.header.stamp.nsec';
info.MatPath{7} = 'features.header.frame_id';
info.MatPath{8} = 'features.lines';
info.MatPath{9} = 'features.lines.pts';
info.MatPath{10} = 'features.descriptors';
info.MatPath{11} = 'features.confidences';
info.MatPath{12} = 'features.descriptor_dim';
