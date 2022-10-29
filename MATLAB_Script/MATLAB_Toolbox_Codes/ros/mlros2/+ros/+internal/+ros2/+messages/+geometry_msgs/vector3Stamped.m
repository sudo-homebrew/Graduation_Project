function [data, info] = vector3Stamped
%Vector3Stamped gives an empty data for geometry_msgs/Vector3Stamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/Vector3Stamped';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.vector, info.vector] = ros.internal.ros2.messages.geometry_msgs.vector3;
info.vector.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/Vector3Stamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,9);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'vector';
info.MatPath{7} = 'vector.x';
info.MatPath{8} = 'vector.y';
info.MatPath{9} = 'vector.z';
