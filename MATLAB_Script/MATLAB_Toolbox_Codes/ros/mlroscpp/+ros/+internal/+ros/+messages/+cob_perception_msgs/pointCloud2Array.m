function [data, info] = pointCloud2Array
%PointCloud2Array gives an empty data for cob_perception_msgs/PointCloud2Array

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'cob_perception_msgs/PointCloud2Array';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Segments, info.Segments] = ros.internal.ros.messages.sensor_msgs.pointCloud2;
info.Segments.MLdataType = 'struct';
info.Segments.MaxLen = NaN;
info.Segments.MinLen = 0;
data.Segments = data.Segments([],1);
info.MessageType = 'cob_perception_msgs/PointCloud2Array';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,33);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'segments';
info.MatPath{8} = 'segments.header';
info.MatPath{9} = 'segments.header.seq';
info.MatPath{10} = 'segments.header.stamp';
info.MatPath{11} = 'segments.header.stamp.sec';
info.MatPath{12} = 'segments.header.stamp.nsec';
info.MatPath{13} = 'segments.header.frame_id';
info.MatPath{14} = 'segments.height';
info.MatPath{15} = 'segments.width';
info.MatPath{16} = 'segments.fields';
info.MatPath{17} = 'segments.fields.INT8';
info.MatPath{18} = 'segments.fields.UINT8';
info.MatPath{19} = 'segments.fields.INT16';
info.MatPath{20} = 'segments.fields.UINT16';
info.MatPath{21} = 'segments.fields.INT32';
info.MatPath{22} = 'segments.fields.UINT32';
info.MatPath{23} = 'segments.fields.FLOAT32';
info.MatPath{24} = 'segments.fields.FLOAT64';
info.MatPath{25} = 'segments.fields.name';
info.MatPath{26} = 'segments.fields.offset';
info.MatPath{27} = 'segments.fields.datatype';
info.MatPath{28} = 'segments.fields.count';
info.MatPath{29} = 'segments.is_bigendian';
info.MatPath{30} = 'segments.point_step';
info.MatPath{31} = 'segments.row_step';
info.MatPath{32} = 'segments.data';
info.MatPath{33} = 'segments.is_dense';