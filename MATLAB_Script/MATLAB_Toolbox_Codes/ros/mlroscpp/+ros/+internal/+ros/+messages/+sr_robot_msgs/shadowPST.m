function [data, info] = shadowPST
%ShadowPST gives an empty data for sr_robot_msgs/ShadowPST

% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
data.MessageType = 'sr_robot_msgs/ShadowPST';
[data.Header, info.Header] = ros.internal.ros.messages.std_msgs.header;
info.Header.MLdataType = 'struct';
[data.Pressure, info.Pressure] = ros.internal.ros.messages.ros.default_type('int16',NaN);
[data.Temperature, info.Temperature] = ros.internal.ros.messages.ros.default_type('int16',NaN);
info.MessageType = 'sr_robot_msgs/ShadowPST';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,8);
info.MatPath{1} = 'header';
info.MatPath{2} = 'header.seq';
info.MatPath{3} = 'header.stamp';
info.MatPath{4} = 'header.stamp.sec';
info.MatPath{5} = 'header.stamp.nsec';
info.MatPath{6} = 'header.frame_id';
info.MatPath{7} = 'pressure';
info.MatPath{8} = 'temperature';
