function [data, info] = jointState
%JointState gives an empty data for sensor_msgs/JointState

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sensor_msgs/JointState';
[data.header, info.header] = ros.internal.ros2.messages.std_msgs.header;
info.header.MLdataType = 'struct';
[data.name, info.name] = ros.internal.ros2.messages.ros2.char('string',NaN,NaN,0);
[data.position, info.position] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
[data.velocity, info.velocity] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
[data.effort, info.effort] = ros.internal.ros2.messages.ros2.default_type('double',NaN,0);
info.MessageType = 'sensor_msgs/JointState';
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
info.MatPath{6} = 'name';
info.MatPath{7} = 'position';
info.MatPath{8} = 'velocity';
info.MatPath{9} = 'effort';
