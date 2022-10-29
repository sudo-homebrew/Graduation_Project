function [data, info] = jointJog
%JointJog gives an empty data for control_msgs/JointJog

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'control_msgs/JointJog';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.JointNames, info.JointNames] = ros.internal.ros.messages.ros.char('string',NaN);
[data.Displacements, info.Displacements] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Velocities, info.Velocities] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Duration, info.Duration] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'control_msgs/JointJog';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'joint_names';
info.MatPath{8} = 'displacements';
info.MatPath{9} = 'velocities';
info.MatPath{10} = 'duration';
