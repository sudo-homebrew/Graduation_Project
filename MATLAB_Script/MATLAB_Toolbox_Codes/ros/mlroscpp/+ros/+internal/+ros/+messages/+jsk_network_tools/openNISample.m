function [data, info] = openNISample
%OpenNISample gives an empty data for jsk_network_tools/OpenNISample

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'jsk_network_tools/OpenNISample';
[data.CameraRgbImageRectColor, info.CameraRgbImageRectColor] = ros.internal.ros.messages.sensor_msgs.image;
info.CameraRgbImageRectColor.MLdataType = 'struct';
[data.CameraDepthRegisteredPoints, info.CameraDepthRegisteredPoints] = ros.internal.ros.messages.sensor_msgs.pointCloud2;
info.CameraDepthRegisteredPoints.MLdataType = 'struct';
info.MessageType = 'jsk_network_tools/OpenNISample';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,40);
info.MatPath{1} = 'camera__rgb__image_rect_color';
info.MatPath{2} = 'camera__rgb__image_rect_color.header';
info.MatPath{3} = 'camera__rgb__image_rect_color.header.seq';
info.MatPath{4} = 'camera__rgb__image_rect_color.header.stamp';
info.MatPath{5} = 'camera__rgb__image_rect_color.header.stamp.sec';
info.MatPath{6} = 'camera__rgb__image_rect_color.header.stamp.nsec';
info.MatPath{7} = 'camera__rgb__image_rect_color.header.frame_id';
info.MatPath{8} = 'camera__rgb__image_rect_color.height';
info.MatPath{9} = 'camera__rgb__image_rect_color.width';
info.MatPath{10} = 'camera__rgb__image_rect_color.encoding';
info.MatPath{11} = 'camera__rgb__image_rect_color.is_bigendian';
info.MatPath{12} = 'camera__rgb__image_rect_color.step';
info.MatPath{13} = 'camera__rgb__image_rect_color.data';
info.MatPath{14} = 'camera__depth_registered__points';
info.MatPath{15} = 'camera__depth_registered__points.header';
info.MatPath{16} = 'camera__depth_registered__points.header.seq';
info.MatPath{17} = 'camera__depth_registered__points.header.stamp';
info.MatPath{18} = 'camera__depth_registered__points.header.stamp.sec';
info.MatPath{19} = 'camera__depth_registered__points.header.stamp.nsec';
info.MatPath{20} = 'camera__depth_registered__points.header.frame_id';
info.MatPath{21} = 'camera__depth_registered__points.height';
info.MatPath{22} = 'camera__depth_registered__points.width';
info.MatPath{23} = 'camera__depth_registered__points.fields';
info.MatPath{24} = 'camera__depth_registered__points.fields.INT8';
info.MatPath{25} = 'camera__depth_registered__points.fields.UINT8';
info.MatPath{26} = 'camera__depth_registered__points.fields.INT16';
info.MatPath{27} = 'camera__depth_registered__points.fields.UINT16';
info.MatPath{28} = 'camera__depth_registered__points.fields.INT32';
info.MatPath{29} = 'camera__depth_registered__points.fields.UINT32';
info.MatPath{30} = 'camera__depth_registered__points.fields.FLOAT32';
info.MatPath{31} = 'camera__depth_registered__points.fields.FLOAT64';
info.MatPath{32} = 'camera__depth_registered__points.fields.name';
info.MatPath{33} = 'camera__depth_registered__points.fields.offset';
info.MatPath{34} = 'camera__depth_registered__points.fields.datatype';
info.MatPath{35} = 'camera__depth_registered__points.fields.count';
info.MatPath{36} = 'camera__depth_registered__points.is_bigendian';
info.MatPath{37} = 'camera__depth_registered__points.point_step';
info.MatPath{38} = 'camera__depth_registered__points.row_step';
info.MatPath{39} = 'camera__depth_registered__points.data';
info.MatPath{40} = 'camera__depth_registered__points.is_dense';