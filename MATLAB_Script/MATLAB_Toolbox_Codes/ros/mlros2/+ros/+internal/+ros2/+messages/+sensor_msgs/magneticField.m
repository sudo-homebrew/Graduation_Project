function [data, info] = magneticField
%MagneticField gives an empty data for sensor_msgs/MagneticField

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/MagneticField';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.magnetic_field, info.magnetic_field] = ros.internal.ros2.messages.geometry_msgs.vector3;
info.magnetic_field.MLdataType = 'struct';
[data.magnetic_field_covariance, info.magnetic_field_covariance] = ros.internal.ros2.messages.ros2.default_type('double',9,0);
info.MessageType = 'sensor_msgs/MagneticField';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'magnetic_field';
info.MatPath{7} = 'magnetic_field.x';
info.MatPath{8} = 'magnetic_field.y';
info.MatPath{9} = 'magnetic_field.z';
info.MatPath{10} = 'magnetic_field_covariance';
