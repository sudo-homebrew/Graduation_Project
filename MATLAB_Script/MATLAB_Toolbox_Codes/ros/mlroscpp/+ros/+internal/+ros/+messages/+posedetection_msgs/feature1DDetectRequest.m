function [data, info] = feature1DDetectRequest
%Feature1DDetect gives an empty data for posedetection_msgs/Feature1DDetectRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'posedetection_msgs/Feature1DDetectRequest';
[data.Image, info.Image] = ros.internal.ros.messages.sensor_msgs.image;
info.Image.MLdataType = 'struct';
info.MessageType = 'posedetection_msgs/Feature1DDetectRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,13);
info.MatPath{1} = 'image';
info.MatPath{2} = 'image.header';
info.MatPath{3} = 'image.header.seq';
info.MatPath{4} = 'image.header.stamp';
info.MatPath{5} = 'image.header.stamp.sec';
info.MatPath{6} = 'image.header.stamp.nsec';
info.MatPath{7} = 'image.header.frame_id';
info.MatPath{8} = 'image.height';
info.MatPath{9} = 'image.width';
info.MatPath{10} = 'image.encoding';
info.MatPath{11} = 'image.is_bigendian';
info.MatPath{12} = 'image.step';
info.MatPath{13} = 'image.data';
