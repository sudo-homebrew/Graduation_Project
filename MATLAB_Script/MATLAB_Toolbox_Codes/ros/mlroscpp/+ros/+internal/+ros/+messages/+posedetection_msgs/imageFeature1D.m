function [data, info] = imageFeature1D
%ImageFeature1D gives an empty data for posedetection_msgs/ImageFeature1D

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'posedetection_msgs/ImageFeature1D';
[data.Image, info.Image] = ros.internal.ros.messages.sensor_msgs.image;
info.Image.MLdataType = 'struct';
[data.Info, info.Info] = ros.internal.ros.messages.sensor_msgs.cameraInfo;
info.Info.MLdataType = 'struct';
[data.Features, info.Features] = ros.internal.ros.messages.posedetection_msgs.feature1D;
info.Features.MLdataType = 'struct';
info.MessageType = 'posedetection_msgs/ImageFeature1D';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,47);
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
info.MatPath{14} = 'info';
info.MatPath{15} = 'info.header';
info.MatPath{16} = 'info.header.seq';
info.MatPath{17} = 'info.header.stamp';
info.MatPath{18} = 'info.header.stamp.sec';
info.MatPath{19} = 'info.header.stamp.nsec';
info.MatPath{20} = 'info.header.frame_id';
info.MatPath{21} = 'info.height';
info.MatPath{22} = 'info.width';
info.MatPath{23} = 'info.distortion_model';
info.MatPath{24} = 'info.D';
info.MatPath{25} = 'info.K';
info.MatPath{26} = 'info.R';
info.MatPath{27} = 'info.P';
info.MatPath{28} = 'info.binning_x';
info.MatPath{29} = 'info.binning_y';
info.MatPath{30} = 'info.roi';
info.MatPath{31} = 'info.roi.x_offset';
info.MatPath{32} = 'info.roi.y_offset';
info.MatPath{33} = 'info.roi.height';
info.MatPath{34} = 'info.roi.width';
info.MatPath{35} = 'info.roi.do_rectify';
info.MatPath{36} = 'features';
info.MatPath{37} = 'features.header';
info.MatPath{38} = 'features.header.seq';
info.MatPath{39} = 'features.header.stamp';
info.MatPath{40} = 'features.header.stamp.sec';
info.MatPath{41} = 'features.header.stamp.nsec';
info.MatPath{42} = 'features.header.frame_id';
info.MatPath{43} = 'features.lines';
info.MatPath{44} = 'features.lines.pts';
info.MatPath{45} = 'features.descriptors';
info.MatPath{46} = 'features.confidences';
info.MatPath{47} = 'features.descriptor_dim';
