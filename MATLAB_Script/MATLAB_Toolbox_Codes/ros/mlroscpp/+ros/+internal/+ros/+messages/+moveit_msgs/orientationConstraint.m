function [data, info] = orientationConstraint
%OrientationConstraint gives an empty data for moveit_msgs/OrientationConstraint

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'moveit_msgs/OrientationConstraint';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Orientation, info.Orientation] = ros.internal.ros.messages.geometry_msgs.quaternion;
info.Orientation.MLdataType = 'struct';
[data.LinkName, info.LinkName] = ros.internal.ros.messages.ros.char('string',0);
[data.AbsoluteXAxisTolerance, info.AbsoluteXAxisTolerance] = ros.internal.ros.messages.ros.default_type('double',1);
[data.AbsoluteYAxisTolerance, info.AbsoluteYAxisTolerance] = ros.internal.ros.messages.ros.default_type('double',1);
[data.AbsoluteZAxisTolerance, info.AbsoluteZAxisTolerance] = ros.internal.ros.messages.ros.default_type('double',1);
[data.Parameterization, info.Parameterization] = ros.internal.ros.messages.ros.default_type('uint8',1);
[data.XYZEULERANGLES, info.XYZEULERANGLES] = ros.internal.ros.messages.ros.default_type('uint8',1, 0);
[data.ROTATIONVECTOR, info.ROTATIONVECTOR] = ros.internal.ros.messages.ros.default_type('uint8',1, 1);
[data.Weight, info.Weight] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'moveit_msgs/OrientationConstraint';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,19);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'orientation';
info.MatPath{8} = 'orientation.x';
info.MatPath{9} = 'orientation.y';
info.MatPath{10} = 'orientation.z';
info.MatPath{11} = 'orientation.w';
info.MatPath{12} = 'link_name';
info.MatPath{13} = 'absolute_x_axis_tolerance';
info.MatPath{14} = 'absolute_y_axis_tolerance';
info.MatPath{15} = 'absolute_z_axis_tolerance';
info.MatPath{16} = 'parameterization';
info.MatPath{17} = 'XYZ_EULER_ANGLES';
info.MatPath{18} = 'ROTATION_VECTOR';
info.MatPath{19} = 'weight';