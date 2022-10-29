function [data, info] = wrenchStamped
%WrenchStamped gives an empty data for geometry_msgs/WrenchStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/WrenchStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Wrench, info.Wrench] = ros.internal.ros.messages.geometry_msgs.wrench;
info.Wrench.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/WrenchStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,15);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'wrench';
info.MatPath{8} = 'wrench.force';
info.MatPath{9} = 'wrench.force.x';
info.MatPath{10} = 'wrench.force.y';
info.MatPath{11} = 'wrench.force.z';
info.MatPath{12} = 'wrench.torque';
info.MatPath{13} = 'wrench.torque.x';
info.MatPath{14} = 'wrench.torque.y';
info.MatPath{15} = 'wrench.torque.z';
