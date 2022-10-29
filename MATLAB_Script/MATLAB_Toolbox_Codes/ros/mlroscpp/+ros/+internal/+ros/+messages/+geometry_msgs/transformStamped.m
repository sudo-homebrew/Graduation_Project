function [data, info] = transformStamped
%TransformStamped gives an empty data for geometry_msgs/TransformStamped

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'geometry_msgs/TransformStamped';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.ChildFrameId, info.ChildFrameId] = ros.internal.ros.messages.ros.char('string',0);
[data.Transform, info.Transform] = ros.internal.ros.messages.geometry_msgs.transform;
info.Transform.MLdataType = 'struct';
info.MessageType = 'geometry_msgs/TransformStamped';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,17);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'child_frame_id';
info.MatPath{8} = 'transform';
info.MatPath{9} = 'transform.translation';
info.MatPath{10} = 'transform.translation.x';
info.MatPath{11} = 'transform.translation.y';
info.MatPath{12} = 'transform.translation.z';
info.MatPath{13} = 'transform.rotation';
info.MatPath{14} = 'transform.rotation.x';
info.MatPath{15} = 'transform.rotation.y';
info.MatPath{16} = 'transform.rotation.z';
info.MatPath{17} = 'transform.rotation.w';
