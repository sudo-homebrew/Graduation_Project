function [data, info] = magneticField
%MagneticField gives an empty data for sensor_msgs/MagneticField

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/MagneticField';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.MagneticField_, info.MagneticField_] = ros.internal.ros.messages.geometry_msgs.vector3;
info.MagneticField_.MLdataType = 'struct';
[data.MagneticFieldCovariance, info.MagneticFieldCovariance] = ros.internal.ros.messages.ros.default_type('double',9);
info.MessageType = 'sensor_msgs/MagneticField';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'magnetic_field';
info.MatPath{8} = 'magnetic_field.x';
info.MatPath{9} = 'magnetic_field.y';
info.MatPath{10} = 'magnetic_field.z';
info.MatPath{11} = 'magnetic_field_covariance';
