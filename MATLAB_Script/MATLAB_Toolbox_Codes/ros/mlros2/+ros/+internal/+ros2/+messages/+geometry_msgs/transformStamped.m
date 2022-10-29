function [data, info] = transformStamped
%TransformStamped gives an empty data for geometry_msgs/TransformStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/TransformStamped';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.child_frame_id, info.child_frame_id] = ros.internal.ros2.messages.ros2.char('string',1,NaN,0);
[data.transform, info.transform] = ros.internal.ros2.messages.geometry_msgs.transform;
info.transform.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/TransformStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,16);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'child_frame_id';
info.MatPath{7} = 'transform';
info.MatPath{8} = 'transform.translation';
info.MatPath{9} = 'transform.translation.x';
info.MatPath{10} = 'transform.translation.y';
info.MatPath{11} = 'transform.translation.z';
info.MatPath{12} = 'transform.rotation';
info.MatPath{13} = 'transform.rotation.x';
info.MatPath{14} = 'transform.rotation.y';
info.MatPath{15} = 'transform.rotation.z';
info.MatPath{16} = 'transform.rotation.w';
