function [data, info] = wrenchStamped
%WrenchStamped gives an empty data for geometry_msgs/WrenchStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/WrenchStamped';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.wrench, info.wrench] = ros.internal.ros2.messages.geometry_msgs.wrench;
info.wrench.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/WrenchStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,14);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.stamp';
info.MatPath{3} = 'header.stamp.sec';
info.MatPath{4} = 'header.stamp.nanosec';
info.MatPath{5} = 'header.frame_id';
info.MatPath{6} = 'wrench';
info.MatPath{7} = 'wrench.force';
info.MatPath{8} = 'wrench.force.x';
info.MatPath{9} = 'wrench.force.y';
info.MatPath{10} = 'wrench.force.z';
info.MatPath{11} = 'wrench.torque';
info.MatPath{12} = 'wrench.torque.x';
info.MatPath{13} = 'wrench.torque.y';
info.MatPath{14} = 'wrench.torque.z';
