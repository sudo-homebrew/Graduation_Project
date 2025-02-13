function [data, info] = imagePercept
%ImagePercept gives an empty data for hector_worldmodel_msgs/ImagePercept

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'hector_worldmodel_msgs/ImagePercept';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.CameraInfo, info.CameraInfo] = ros.internal.ros.messages.sensor_msgs.cameraInfo;
info.CameraInfo.MLdataType = 'struct';
[data.X, info.X] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Y, info.Y] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Width, info.Width] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Height, info.Height] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Distance, info.Distance] = ros.internal.ros.messages.ros.default_type('single',1);
[data.Info, info.Info] = ros.internal.ros.messages.hector_worldmodel_msgs.perceptInfo;
info.Info.MLdataType = 'struct';
info.MessageType = 'hector_worldmodel_msgs/ImagePercept';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,39);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'camera_info';
info.MatPath{8} = 'camera_info.header';
info.MatPath{9} = 'camera_info.header.seq';
info.MatPath{10} = 'camera_info.header.stamp';
info.MatPath{11} = 'camera_info.header.stamp.sec';
info.MatPath{12} = 'camera_info.header.stamp.nsec';
info.MatPath{13} = 'camera_info.header.frame_id';
info.MatPath{14} = 'camera_info.height';
info.MatPath{15} = 'camera_info.width';
info.MatPath{16} = 'camera_info.distortion_model';
info.MatPath{17} = 'camera_info.D';
info.MatPath{18} = 'camera_info.K';
info.MatPath{19} = 'camera_info.R';
info.MatPath{20} = 'camera_info.P';
info.MatPath{21} = 'camera_info.binning_x';
info.MatPath{22} = 'camera_info.binning_y';
info.MatPath{23} = 'camera_info.roi';
info.MatPath{24} = 'camera_info.roi.x_offset';
info.MatPath{25} = 'camera_info.roi.y_offset';
info.MatPath{26} = 'camera_info.roi.height';
info.MatPath{27} = 'camera_info.roi.width';
info.MatPath{28} = 'camera_info.roi.do_rectify';
info.MatPath{29} = 'x';
info.MatPath{30} = 'y';
info.MatPath{31} = 'width';
info.MatPath{32} = 'height';
info.MatPath{33} = 'distance';
info.MatPath{34} = 'info';
info.MatPath{35} = 'info.class_id';
info.MatPath{36} = 'info.class_support';
info.MatPath{37} = 'info.object_id';
info.MatPath{38} = 'info.object_support';
info.MatPath{39} = 'info.name';
