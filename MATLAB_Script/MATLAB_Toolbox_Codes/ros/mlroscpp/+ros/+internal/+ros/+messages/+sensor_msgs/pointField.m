function [data, info] = pointField
%PointField gives an empty data for sensor_msgs/PointField

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/PointField';
[data.INT8, info.INT8] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.UINT8, info.UINT8] = ros.internal.ros.messages.ros.default_type('uint8',1, 2);
[data.INT16, info.INT16] = ros.internal.ros.messages.ros.default_type('uint8',1, 3);
[data.UINT16, info.UINT16] = ros.internal.ros.messages.ros.default_type('uint8',1, 4);
[data.INT32, info.INT32] = ros.internal.ros.messages.ros.default_type('uint8',1, 5);
[data.UINT32, info.UINT32] = ros.internal.ros.messages.ros.default_type('uint8',1, 6);
[data.FLOAT32, info.FLOAT32] = ros.internal.ros.messages.ros.default_type('uint8',1, 7);
[data.FLOAT64, info.FLOAT64] = ros.internal.ros.messages.ros.default_type('uint8',1, 8);
[data.Name, info.Name] = ros.internal.ros.messages.ros.char('string',0);
[data.Offset, info.Offset] = ros.internal.ros.messages.ros.default_type('uint32',1);
[data.Datatype, info.Datatype] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.Count, info.Count] = ros.internal.ros.messages.ros.default_type('uint32',1);
info.MessageType = 'sensor_msgs/PointField';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'INT8';
info.MatPath{2} = 'UINT8';
info.MatPath{3} = 'INT16';
info.MatPath{4} = 'UINT16';
info.MatPath{5} = 'INT32';
info.MatPath{6} = 'UINT32';
info.MatPath{7} = 'FLOAT32';
info.MatPath{8} = 'FLOAT64';
info.MatPath{9} = 'name';
info.MatPath{10} = 'offset';
info.MatPath{11} = 'datatype';
info.MatPath{12} = 'count';