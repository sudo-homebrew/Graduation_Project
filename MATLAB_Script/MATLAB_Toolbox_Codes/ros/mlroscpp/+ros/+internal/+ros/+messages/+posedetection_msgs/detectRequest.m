function [data, info] = detectRequest
%Detect gives an empty data for posedetection_msgs/DetectRequest

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'posedetection_msgs/DetectRequest';
[data.Image, info.Image] = ros.internal.ros.messages.sensor_msgs.image;
info.Image.MLdataType = 'struct';
[data.CameraInfo, info.CameraInfo] = ros.internal.ros.messages.sensor_msgs.cameraInfo;
info.CameraInfo.MLdataType = 'struct';
info.MessageType = 'posedetection_msgs/DetectRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,35);
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
info.MatPath{14} = 'camera_info';
info.MatPath{15} = 'camera_info.header';
info.MatPath{16} = 'camera_info.header.seq';
info.MatPath{17} = 'camera_info.header.stamp';
info.MatPath{18} = 'camera_info.header.stamp.sec';
info.MatPath{19} = 'camera_info.header.stamp.nsec';
info.MatPath{20} = 'camera_info.header.frame_id';
info.MatPath{21} = 'camera_info.height';
info.MatPath{22} = 'camera_info.width';
info.MatPath{23} = 'camera_info.distortion_model';
info.MatPath{24} = 'camera_info.D';
info.MatPath{25} = 'camera_info.K';
info.MatPath{26} = 'camera_info.R';
info.MatPath{27} = 'camera_info.P';
info.MatPath{28} = 'camera_info.binning_x';
info.MatPath{29} = 'camera_info.binning_y';
info.MatPath{30} = 'camera_info.roi';
info.MatPath{31} = 'camera_info.roi.x_offset';
info.MatPath{32} = 'camera_info.roi.y_offset';
info.MatPath{33} = 'camera_info.roi.height';
info.MatPath{34} = 'camera_info.roi.width';
info.MatPath{35} = 'camera_info.roi.do_rectify';
