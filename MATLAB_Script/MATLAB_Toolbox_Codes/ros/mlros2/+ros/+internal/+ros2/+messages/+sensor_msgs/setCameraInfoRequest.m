function [data, info] = setCameraInfoRequest
%SetCameraInfo gives an empty data for sensor_msgs/SetCameraInfoRequest

% Copyright 2019-2021 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/SetCameraInfoRequest';
[data.camera_info, info.camera_info] = ros.internal.ros2.messages.sensor_msgs.cameraInfo;
info.camera_info.MLdataType = 'struct';
info.MessageType = 'sensor_msgs/SetCameraInfoRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,21);
info.MatPath{1} = 'camera_info';
info.MatPath{2} = 'camera_info.header';
info.MatPath{3} = 'camera_info.header.stamp';
info.MatPath{4} = 'camera_info.header.stamp.sec';
info.MatPath{5} = 'camera_info.header.stamp.nanosec';
info.MatPath{6} = 'camera_info.header.frame_id';
info.MatPath{7} = 'camera_info.height';
info.MatPath{8} = 'camera_info.width';
info.MatPath{9} = 'camera_info.distortion_model';
info.MatPath{10} = 'camera_info.d';
info.MatPath{11} = 'camera_info.k';
info.MatPath{12} = 'camera_info.r';
info.MatPath{13} = 'camera_info.p';
info.MatPath{14} = 'camera_info.binning_x';
info.MatPath{15} = 'camera_info.binning_y';
info.MatPath{16} = 'camera_info.roi';
info.MatPath{17} = 'camera_info.roi.x_offset';
info.MatPath{18} = 'camera_info.roi.y_offset';
info.MatPath{19} = 'camera_info.roi.height';
info.MatPath{20} = 'camera_info.roi.width';
info.MatPath{21} = 'camera_info.roi.do_rectify';
