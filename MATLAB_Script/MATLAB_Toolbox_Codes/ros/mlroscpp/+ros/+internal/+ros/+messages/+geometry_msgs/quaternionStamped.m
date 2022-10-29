function [data, info] = quaternionStamped
%QuaternionStamped gives an empty data for geometry_msgs/QuaternionStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/QuaternionStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Quaternion, info.Quaternion] = ros.internal.ros.messages.geometry_msgs.quaternion;
info.Quaternion.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/QuaternionStamped';
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
info.MatPath{7} = 'quaternion';
info.MatPath{8} = 'quaternion.x';
info.MatPath{9} = 'quaternion.y';
info.MatPath{10} = 'quaternion.z';
info.MatPath{11} = 'quaternion.w';
