function [data, info] = getSegmentedLineResponse
%GetSegmentedLine gives an empty data for sr_robot_msgs/GetSegmentedLineResponse

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/GetSegmentedLineResponse';
[data.LineCloud, info.LineCloud] = ros.internal.ros.messages.sensor_msgs.pointCloud2;
info.LineCloud.MLdataType = 'struct';
info.MessageType = 'sr_robot_msgs/GetSegmentedLineResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,27);
info.MatPath{1} = 'line_cloud';
info.MatPath{2} = 'line_cloud.header';
info.MatPath{3} = 'line_cloud.header.seq';
info.MatPath{4} = 'line_cloud.header.stamp';
info.MatPath{5} = 'line_cloud.header.stamp.sec';
info.MatPath{6} = 'line_cloud.header.stamp.nsec';
info.MatPath{7} = 'line_cloud.header.frame_id';
info.MatPath{8} = 'line_cloud.height';
info.MatPath{9} = 'line_cloud.width';
info.MatPath{10} = 'line_cloud.fields';
info.MatPath{11} = 'line_cloud.fields.INT8';
info.MatPath{12} = 'line_cloud.fields.UINT8';
info.MatPath{13} = 'line_cloud.fields.INT16';
info.MatPath{14} = 'line_cloud.fields.UINT16';
info.MatPath{15} = 'line_cloud.fields.INT32';
info.MatPath{16} = 'line_cloud.fields.UINT32';
info.MatPath{17} = 'line_cloud.fields.FLOAT32';
info.MatPath{18} = 'line_cloud.fields.FLOAT64';
info.MatPath{19} = 'line_cloud.fields.name';
info.MatPath{20} = 'line_cloud.fields.offset';
info.MatPath{21} = 'line_cloud.fields.datatype';
info.MatPath{22} = 'line_cloud.fields.count';
info.MatPath{23} = 'line_cloud.is_bigendian';
info.MatPath{24} = 'line_cloud.point_step';
info.MatPath{25} = 'line_cloud.row_step';
info.MatPath{26} = 'line_cloud.data';
info.MatPath{27} = 'line_cloud.is_dense';